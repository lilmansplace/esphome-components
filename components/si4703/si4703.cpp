#include "si4703.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome::si4703 {

static const char *const TAG = "si4703";

// Register indexes (16-bit registers)
static constexpr uint8_t REG_DEVICEID = 0x00;
static constexpr uint8_t REG_CHIPID = 0x01;
static constexpr uint8_t REG_POWERCFG = 0x02;
static constexpr uint8_t REG_CHANNEL = 0x03;
static constexpr uint8_t REG_SYSCONFIG1 = 0x04;
static constexpr uint8_t REG_SYSCONFIG2 = 0x05;
static constexpr uint8_t REG_SYSCONFIG3 = 0x06;
static constexpr uint8_t REG_TEST1 = 0x07;
static constexpr uint8_t REG_STATUSRSSI = 0x0A;
static constexpr uint8_t REG_READCHAN = 0x0B;
static constexpr uint8_t REG_RDSB = 0x0D;
static constexpr uint8_t REG_RDSC = 0x0E;
static constexpr uint8_t REG_RDSD = 0x0F;

static constexpr uint16_t EXPECTED_DEVICEID = 0x1242;

// POWERCFG (0x02)
static constexpr uint16_t PWR_DMUTE = 1 << 14;
static constexpr uint16_t PWR_MONO = 1 << 13;
static constexpr uint16_t PWR_RDSM = 1 << 11;  // RDS verbose mode: block error levels in READCHAN
static constexpr uint16_t PWR_SKMODE = 1 << 10;
static constexpr uint16_t PWR_SEEKUP = 1 << 9;
static constexpr uint16_t PWR_SEEK = 1 << 8;
static constexpr uint16_t PWR_ENABLE = 1 << 0;
// CHANNEL (0x03)
static constexpr uint16_t CHAN_TUNE = 1 << 15;
static constexpr uint16_t CHAN_MASK = 0x03FF;
// SYSCONFIG1 (0x04)
static constexpr uint16_t SC1_RDS = 1 << 12;
static constexpr uint16_t SC1_DE = 1 << 11;
// SYSCONFIG2 (0x05): SEEKTH[15:8] BAND[7:6] SPACE[5:4] VOLUME[3:0]
static constexpr uint16_t SC2_VOLUME_MASK = 0x000F;
// SYSCONFIG3 (0x06): SKSNR[7:4] = 4, SKCNT[3:0] = 8 (AN230 recommended seek settings)
static constexpr uint16_t SC3_SEEK_DEFAULTS = 0x0048;
// TEST1 (0x07): bit 15 XOSCEN, bit 14 AHIZEN, bits 13:0 reserved (must be written back as read)
static constexpr uint16_t TEST1_XOSCEN = 0x8000;
static constexpr uint16_t TEST1_AHIZEN = 0x4000;
// STATUSRSSI (0x0A)
static constexpr uint16_t ST_RDSR = 1 << 15;
static constexpr uint16_t ST_STC = 1 << 14;
static constexpr uint16_t ST_SFBL = 1 << 13;
static constexpr uint16_t ST_ST = 1 << 8;
static constexpr uint16_t ST_RSSI_MASK = 0x00FF;

// Timing
static constexpr uint32_t POLL_INTERVAL_MS = 20;  // RDS groups arrive every ~88 ms
// The datasheet advises keeping the bus quiet while a tune/seek runs, as bus activity can disturb the
// reference oscillator. Tune takes ~60 ms, so poll for STC less often while busy.
static constexpr uint32_t BUSY_POLL_INTERVAL_MS = 60;
static constexpr uint32_t RSSI_PUBLISH_INTERVAL_MS = 2000;
static constexpr uint32_t SETTLE_MS = 500;  // RSSI/stereo settle time after a tune
static constexpr uint32_t TUNE_TIMEOUT_MS = 500;
static constexpr uint32_t SEEK_TIMEOUT_MS = 10000;
static constexpr uint32_t CLEAR_TIMEOUT_MS = 200;
static constexpr uint32_t OSCILLATOR_DELAY_MS = 600;  // AN230: >= 500 ms
static constexpr uint32_t POWERUP_DELAY_MS = 120;     // datasheet powerup time 110 ms
static constexpr uint8_t MAX_INIT_ATTEMPTS = 5;

static constexpr uint8_t RDS_UNCORRECTABLE = 3;

static const char *band_to_str(Band band) {
  switch (band) {
    case BAND_76_108:
      return "76-108 MHz";
    case BAND_76_90:
      return "76-90 MHz";
    default:
      return "87.5-108 MHz";
  }
}

float Si4703Component::get_band_min() const { return this->band_ == BAND_87_5_108 ? 87.5f : 76.0f; }
float Si4703Component::get_band_max() const { return this->band_ == BAND_76_90 ? 90.0f : 108.0f; }
float Si4703Component::get_channel_step() const {
  switch (this->spacing_) {
    case SPACING_200KHZ:
      return 0.2f;
    case SPACING_50KHZ:
      return 0.05f;
    default:
      return 0.1f;
  }
}

void Si4703Component::setup() {
#ifdef USE_NUMBER
  if (this->frequency_number_ != nullptr) {
    this->frequency_number_->traits.set_min_value(this->get_band_min());
    this->frequency_number_->traits.set_max_value(this->get_band_max());
    this->frequency_number_->traits.set_step(this->get_channel_step());
  }
#endif
  this->hardware_reset_();
  // The I2C bus is initialised after this component; continue once it is up.
  this->set_timeout(50, [this]() { this->init_verify_(); });
}

void Si4703Component::hardware_reset_() {
  // The Si4702/03 latches its bus mode on the rising edge of RST. 2-wire (I2C) mode requires
  // SEN high and SDIO low at that moment. Breakout boards usually pull SDIO up, so drive it low.
  this->reset_pin_->setup();
  this->reset_pin_->digital_write(false);
  if (this->sdio_pin_ != nullptr) {
    this->sdio_pin_->setup();
    this->sdio_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->sdio_pin_->digital_write(false);
  }
  delay(1);
  this->reset_pin_->digital_write(true);
  delay(1);
  if (this->sdio_pin_ != nullptr) {
    this->sdio_pin_->pin_mode(gpio::FLAG_INPUT);  // hand SDIO back to the I2C bus
  }
}

void Si4703Component::init_verify_() {
  this->init_attempts_++;
  if (!this->read_all_() || this->registers_[REG_DEVICEID] != EXPECTED_DEVICEID) {
    ESP_LOGW(TAG, "No response (attempt %u/%u): DEVICEID=0x%04X", this->init_attempts_, MAX_INIT_ATTEMPTS,
             this->registers_[REG_DEVICEID]);
    if (this->init_attempts_ >= MAX_INIT_ATTEMPTS) {
      this->mark_failed(LOG_STR("Si4703 not found; check SEN is high and SDIO is held low during reset"));
      return;
    }
    // Do not pulse RST here: SDIO idles high now, which would latch 3-wire mode.
    this->set_timeout(1000, [this]() { this->init_verify_(); });
    return;
  }
  this->chip_id_ = this->registers_[REG_CHIPID];
  this->test1_reset_ = this->registers_[REG_TEST1];
  // Start the 32.768 kHz crystal oscillator. Read-modify-write: the reserved TEST1 bits differ between
  // silicon revisions and must keep the value the chip reports.
  this->registers_[REG_TEST1] = (this->test1_reset_ & ~TEST1_AHIZEN) | TEST1_XOSCEN;
  ESP_LOGD(TAG, "CHIPID 0x%04X, TEST1 0x%04X -> 0x%04X", this->chip_id_, this->test1_reset_,
           this->registers_[REG_TEST1]);
  if (!this->write_registers_()) {
    this->mark_failed(LOG_STR("Failed to enable oscillator"));
    return;
  }
  this->set_timeout(OSCILLATOR_DELAY_MS, [this]() { this->init_power_up_(); });
}

void Si4703Component::init_power_up_() {
  if (!this->read_all_()) {
    this->mark_failed(LOG_STR("Read failed after oscillator start"));
    return;
  }
  uint16_t pwr = PWR_ENABLE | PWR_RDSM;
  if (!this->muted_)
    pwr |= PWR_DMUTE;
  if (this->mono_)
    pwr |= PWR_MONO;
  this->registers_[REG_POWERCFG] = pwr;
  this->registers_[REG_SYSCONFIG1] = SC1_RDS | (this->de_50us_ ? SC1_DE : 0);
  this->registers_[REG_SYSCONFIG2] = (uint16_t(this->seek_threshold_) << 8) | (uint16_t(this->band_) << 6) |
                                     (uint16_t(this->spacing_) << 4) | (this->volume_ & SC2_VOLUME_MASK);
  this->registers_[REG_SYSCONFIG3] = SC3_SEEK_DEFAULTS;
  if (!this->write_registers_()) {
    this->mark_failed(LOG_STR("Failed to power up"));
    return;
  }
  this->set_timeout(POWERUP_DELAY_MS, [this]() { this->init_finish_(); });
}

void Si4703Component::init_finish_() {
  this->read_all_();
  this->state_ = State::IDLE;
  ESP_LOGI(TAG, "Powered up (CHIPID 0x%04X)", this->registers_[REG_CHIPID]);
#ifdef USE_NUMBER
  if (this->volume_number_ != nullptr)
    this->volume_number_->publish_state(this->volume_);
  if (this->seek_threshold_number_ != nullptr)
    this->seek_threshold_number_->publish_state(this->seek_threshold_);
#endif
#ifdef USE_SWITCH
  if (this->mute_switch_ != nullptr)
    this->mute_switch_->publish_state(this->muted_);
  if (this->mono_switch_ != nullptr)
    this->mono_switch_->publish_state(this->mono_);
#endif
  this->set_frequency(this->target_frequency_);
}

void Si4703Component::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Si4703 FM Tuner:\n"
                "  Band: %s, spacing %.0f kHz, de-emphasis %s\n"
                "  Initial frequency: %.2f MHz, volume %u",
                band_to_str(this->band_), this->get_channel_step() * 1000.0f, this->de_50us_ ? "50 us" : "75 us",
                this->target_frequency_, this->volume_);
  LOG_I2C_DEVICE(this);
  if (this->chip_id_ != 0) {
    ESP_LOGCONFIG(TAG, "  CHIPID: 0x%04X (firmware %u), TEST1 at reset: 0x%04X", this->chip_id_,
                  this->chip_id_ & 0x3F, this->test1_reset_);
  }
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  SDIO Pin: ", this->sdio_pin_);
  if (this->sdio_pin_ == nullptr) {
    ESP_LOGW(TAG, "  No sdio_pin set: the chip may start in 3-wire mode on boards that pull SDIO high");
  }
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Frequency", this->frequency_sensor_);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
#endif
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Stereo", this->stereo_binary_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Station Name", this->station_name_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Radio Text", this->radio_text_text_sensor_);
#endif
}

// ------------------------------------------------------------------ loop / state machine

void Si4703Component::loop() {
  if (this->state_ == State::INIT)
    return;
  const uint32_t now = App.get_loop_component_start_time();
  const bool busy = this->state_ == State::TUNING || this->state_ == State::SEEKING;
  if (busy && now - this->op_started_ < BUSY_POLL_INTERVAL_MS)
    return;
  if (now - this->last_poll_ < (busy ? BUSY_POLL_INTERVAL_MS : POLL_INTERVAL_MS))
    return;
  this->last_poll_ = now;
  if (!this->read_status_())
    return;
  const bool stc = (this->registers_[REG_STATUSRSSI] & ST_STC) != 0;

  switch (this->state_) {
    case State::TUNING:
    case State::SEEKING: {
      const bool seeking = this->state_ == State::SEEKING;
      const uint32_t timeout = seeking ? SEEK_TIMEOUT_MS : TUNE_TIMEOUT_MS;
      if (!stc && now - this->op_started_ < timeout)
        break;
      this->last_op_seek_ = seeking;
      this->stc_rssi_ = this->registers_[REG_STATUSRSSI] & ST_RSSI_MASK;
      this->stc_band_limit_ = !stc || (this->registers_[REG_STATUSRSSI] & ST_SFBL) != 0;
      if (!stc) {
        ESP_LOGW(TAG, "%s timed out - is the 32.768 kHz crystal oscillating?", seeking ? "Seek" : "Tune");
      } else if (seeking && this->stc_band_limit_ && !this->scanning_) {
        ESP_LOGD(TAG, "Seek found no station above the threshold");
      }
      this->registers_[REG_POWERCFG] &= ~PWR_SEEK;
      this->registers_[REG_CHANNEL] &= ~CHAN_TUNE;
      this->write_registers_();
      this->state_ = State::CLEARING;
      this->op_started_ = now;
      break;
    }
    case State::CLEARING:
      if (stc && now - this->op_started_ < CLEAR_TIMEOUT_MS)
        break;
      this->state_ = State::IDLE;
      if (this->scanning_) {
        this->scan_step_();
      } else {
        this->complete_tune_();
      }
      break;
    case State::IDLE:
      this->update_from_status_();
      this->process_rds_();
      break;
    default:
      break;
  }
}

void Si4703Component::update_from_status_() {
  const uint32_t now = App.get_loop_component_start_time();
  if (int32_t(now - this->settle_until_) < 0)
    return;  // still settling after a tune

  const bool stereo = (this->registers_[REG_STATUSRSSI] & ST_ST) != 0;
#ifdef USE_BINARY_SENSOR
  if (this->stereo_binary_sensor_ != nullptr && (stereo != this->stereo_ || !this->stereo_binary_sensor_->has_state()))
    this->stereo_binary_sensor_->publish_state(stereo);
#endif
  this->stereo_ = stereo;
  this->rssi_ = this->registers_[REG_STATUSRSSI] & ST_RSSI_MASK;

  if (this->last_published_rssi_ < 0 || now - this->last_rssi_publish_ >= RSSI_PUBLISH_INTERVAL_MS) {
    if (this->last_published_rssi_ < 0) {
      ESP_LOGI(TAG, "Tuned to %.2f MHz, RSSI %u dBuV, %s", this->frequency_, this->rssi_,
               this->stereo_ ? "stereo" : "mono");
    }
    this->last_rssi_publish_ = now;
    if (this->rssi_ != this->last_published_rssi_) {
      this->last_published_rssi_ = this->rssi_;
#ifdef USE_SENSOR
      if (this->rssi_sensor_ != nullptr)
        this->rssi_sensor_->publish_state(this->rssi_);
#endif
    }
  }
}

void Si4703Component::complete_tune_() {
  const uint16_t chan = this->registers_[REG_READCHAN] & CHAN_MASK;
  const float mhz = this->get_band_min() + chan * this->get_channel_step();
  this->frequency_ = std::round(mhz * 100.0f) / 100.0f;
  this->target_frequency_ = this->frequency_;
  this->settle_until_ = App.get_loop_component_start_time() + SETTLE_MS;
  this->last_published_rssi_ = -1;  // report RSSI once settled
  this->clear_rds_();
  this->publish_tuned_();
}

void Si4703Component::publish_tuned_() {
#ifdef USE_SENSOR
  if (this->frequency_sensor_ != nullptr)
    this->frequency_sensor_->publish_state(this->frequency_);
#endif
#ifdef USE_NUMBER
  if (this->frequency_number_ != nullptr)
    this->frequency_number_->publish_state(this->frequency_);
#endif
}

// ------------------------------------------------------------------ control

void Si4703Component::set_frequency(float mhz) {
  if (this->scanning_) {
    ESP_LOGI(TAG, "Scan cancelled");
    this->scanning_ = false;
    this->registers_[REG_POWERCFG] &= ~PWR_SKMODE;
  }
  const float lo = this->get_band_min();
  const float hi = this->get_band_max();
  if (mhz < lo || mhz > hi) {
    ESP_LOGW(TAG, "%.2f MHz is outside the %s band", mhz, band_to_str(this->band_));
    mhz = std::clamp(mhz, lo, hi);
  }
  this->target_frequency_ = mhz;
  if (this->state_ == State::INIT)
    return;  // applied when initialisation finishes
  this->tune_(mhz);
}

void Si4703Component::tune_(float mhz) {
  const float lo = this->get_band_min();
  const uint16_t chan = static_cast<uint16_t>(std::lround((mhz - lo) / this->get_channel_step()));
  ESP_LOGD(TAG, "Tuning to %.2f MHz (channel %u)", mhz, chan);
  this->registers_[REG_POWERCFG] &= ~PWR_SEEK;  // abort a running seek
  this->registers_[REG_CHANNEL] = CHAN_TUNE | (chan & CHAN_MASK);
  if (!this->write_registers_())
    return;
  this->state_ = State::TUNING;
  this->op_started_ = App.get_loop_component_start_time();
}

void Si4703Component::start_seek_(bool up) {
  if (this->state_ != State::IDLE || this->scanning_) {
    ESP_LOGW(TAG, "Busy, seek ignored");
    return;
  }
  ESP_LOGD(TAG, "Seeking %s", up ? "up" : "down");
  this->issue_seek_(up, false);
}

void Si4703Component::issue_seek_(bool up, bool stop_at_band_limit) {
  this->registers_[REG_POWERCFG] &= ~(PWR_SKMODE | PWR_SEEKUP);
  if (stop_at_band_limit)
    this->registers_[REG_POWERCFG] |= PWR_SKMODE;
  if (up)
    this->registers_[REG_POWERCFG] |= PWR_SEEKUP;
  this->registers_[REG_POWERCFG] |= PWR_SEEK;
  if (!this->write_registers_())
    return;
  this->state_ = State::SEEKING;
  this->op_started_ = App.get_loop_component_start_time();
}

// ------------------------------------------------------------------ band scan

void Si4703Component::start_scan() {
  if (this->state_ != State::IDLE || this->scanning_) {
    ESP_LOGW(TAG, "Busy, scan ignored");
    return;
  }
  ESP_LOGI(TAG, "Scanning %s (seek threshold %u dBuV)", band_to_str(this->band_), this->seek_threshold_);
  this->scanning_ = true;
  this->scan_count_ = 0;
  if (this->status_text_) {
    this->publish_station_status_(this->status_[0] != '\0' ? this->status_ : "Scanning\u2026");
    this->status_[0] = '\0';
#ifdef USE_TEXT_SENSOR
    if (this->radio_text_text_sensor_ != nullptr)
      this->radio_text_text_sensor_->publish_state("Scanning\u2026");
#endif
  }
  this->scan_return_frequency_ = this->frequency_;
  this->tune_(this->get_band_min());  // start at the bottom, then seek upward to the top
}

void Si4703Component::scan_step_() {
  if (this->last_op_seek_) {
    if (this->stc_band_limit_) {
      this->finish_scan_();
      return;
    }
    const uint16_t chan = this->registers_[REG_READCHAN] & CHAN_MASK;
    const float mhz = std::round((this->get_band_min() + chan * this->get_channel_step()) * 100.0f) / 100.0f;
    if (this->scan_count_ > 0 && this->scan_results_[this->scan_count_ - 1].frequency >= mhz) {
      this->finish_scan_();  // wrapped or no progress
      return;
    }
    this->scan_results_[this->scan_count_++] = {mhz, this->stc_rssi_};
    ESP_LOGD(TAG, "Scan found %.2f MHz, RSSI %u dBuV", mhz, this->stc_rssi_);
#ifdef USE_TEXT_SENSOR
    if (this->status_text_ && this->radio_text_text_sensor_ != nullptr) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Found %u \u00b7 %.1f MHz", this->scan_count_, mhz);
      this->radio_text_text_sensor_->publish_state(buf);
    }
#endif
    if (this->scan_count_ >= MAX_SCAN_RESULTS) {
      this->finish_scan_();
      return;
    }
  }
  this->issue_seek_(true, true);
}

void Si4703Component::finish_scan_() {
  this->scanning_ = false;
  this->registers_[REG_POWERCFG] &= ~PWR_SKMODE;
  // Insertion sort, strongest first
  for (uint8_t i = 1; i < this->scan_count_; i++) {
    const ScanResult item = this->scan_results_[i];
    int j = i - 1;
    while (j >= 0 && this->scan_results_[j].rssi < item.rssi) {
      this->scan_results_[j + 1] = this->scan_results_[j];
      j--;
    }
    this->scan_results_[j + 1] = item;
  }
  ESP_LOGI(TAG, "Scan complete: %u stations", this->scan_count_);
  this->tune_(this->scan_return_frequency_ > 0 ? this->scan_return_frequency_ : this->get_band_min());
  // Run user automations in their own loop pass so this one stays short
  this->defer([this]() { this->scan_complete_callback_.call(); });
}

void Si4703Component::set_volume(uint8_t volume) {
  this->volume_ = volume > 15 ? 15 : volume;
  if (this->state_ != State::INIT) {
    this->registers_[REG_SYSCONFIG2] = (this->registers_[REG_SYSCONFIG2] & ~SC2_VOLUME_MASK) | this->volume_;
    this->write_registers_();
  }
#ifdef USE_NUMBER
  if (this->volume_number_ != nullptr)
    this->volume_number_->publish_state(this->volume_);
#endif
}

void Si4703Component::set_mute(bool mute) {
  this->muted_ = mute;
  if (this->state_ != State::INIT) {
    if (mute) {
      this->registers_[REG_POWERCFG] &= ~PWR_DMUTE;
    } else {
      this->registers_[REG_POWERCFG] |= PWR_DMUTE;
    }
    this->write_registers_();
  }
#ifdef USE_SWITCH
  if (this->mute_switch_ != nullptr)
    this->mute_switch_->publish_state(mute);
#endif
}

void Si4703Component::set_mono(bool mono) {
  this->mono_ = mono;
  if (this->state_ != State::INIT) {
    if (mono) {
      this->registers_[REG_POWERCFG] |= PWR_MONO;
    } else {
      this->registers_[REG_POWERCFG] &= ~PWR_MONO;
    }
    this->write_registers_();
  }
#ifdef USE_SWITCH
  if (this->mono_switch_ != nullptr)
    this->mono_switch_->publish_state(mono);
#endif
}

void Si4703Component::set_seek_threshold(uint8_t threshold) {
  this->seek_threshold_ = threshold > 127 ? 127 : threshold;
  if (this->state_ != State::INIT) {
    this->registers_[REG_SYSCONFIG2] = (this->registers_[REG_SYSCONFIG2] & 0x00FF) | (uint16_t(this->seek_threshold_) << 8);
    this->write_registers_();
  }
#ifdef USE_NUMBER
  if (this->seek_threshold_number_ != nullptr)
    this->seek_threshold_number_->publish_state(this->seek_threshold_);
#endif
}

// ------------------------------------------------------------------ I2C

bool Si4703Component::read_all_() {
  uint8_t buf[32];
  if (this->read(buf, sizeof(buf)) != i2c::ERROR_OK)
    return false;
  // Reads start at 0x0A and wrap: 0x0A..0x0F, then 0x00..0x09
  uint8_t reg = REG_STATUSRSSI;
  for (size_t i = 0; i < sizeof(buf); i += 2) {
    this->registers_[reg] = encode_uint16(buf[i], buf[i + 1]);
    reg = (reg + 1) & 0x0F;
  }
  return true;
}

bool Si4703Component::read_status_() {
  uint8_t buf[12];  // 0x0A..0x0F only
  if (this->read(buf, sizeof(buf)) != i2c::ERROR_OK)
    return false;
  for (uint8_t i = 0; i < 6; i++)
    this->registers_[REG_STATUSRSSI + i] = encode_uint16(buf[2 * i], buf[2 * i + 1]);
  return true;
}

bool Si4703Component::write_registers_() {
  // Writes always start at 0x02; write 0x02..0x07
  uint8_t buf[12];
  for (uint8_t i = 0; i < 6; i++) {
    buf[2 * i] = this->registers_[REG_POWERCFG + i] >> 8;
    buf[2 * i + 1] = this->registers_[REG_POWERCFG + i] & 0xFF;
  }
  if (this->write(buf, sizeof(buf)) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C write failed");
    return false;
  }
  return true;
}

// ------------------------------------------------------------------ RDS

static char rds_char(uint8_t c) { return (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : ' '; }

// Copy src (up to len chars, stops at NUL) into dst trimming surrounding spaces.
static void trim_copy(const char *src, size_t len, char *dst) {
  size_t start = 0;
  size_t end = strnlen(src, len);
  while (start < end && src[start] == ' ')
    start++;
  while (end > start && src[end - 1] == ' ')
    end--;
  memcpy(dst, src + start, end - start);
  dst[end - start] = '\0';
}

void Si4703Component::clear_rds_() {
  memset(this->ps_, 0, sizeof(this->ps_));
  memset(this->ps_candidate_, 0, sizeof(this->ps_candidate_));
  memset(this->ps_published_, 0, sizeof(this->ps_published_));
  memset(this->rt_, 0, sizeof(this->rt_));
  memset(this->rt_candidate_, 0, sizeof(this->rt_candidate_));
  memset(this->rt_published_, 0, sizeof(this->rt_published_));
  this->ps_seen_ = 0;
  this->rt_seen_ = 0;
  this->rt_ab_ = -1;
#ifdef USE_TEXT_SENSOR
  if (this->radio_text_text_sensor_ != nullptr)
    this->radio_text_text_sensor_->publish_state("");
  if (!this->status_text_) {
    if (this->station_name_text_sensor_ != nullptr)
      this->station_name_text_sensor_->publish_state("");
    return;
  }
  if (this->status_[0] != '\0') {
    this->publish_station_status_(this->status_);
    this->status_[0] = '\0';  // shown once, for this tune
  } else {
    char buf[16];
    snprintf(buf, sizeof(buf), this->get_channel_step() < 0.1f ? "%.2f FM" : "%.1f FM", this->frequency_);
    this->publish_station_status_(buf);
  }
#endif
}

void Si4703Component::publish_station_status_(const char *text) {
#ifdef USE_TEXT_SENSOR
  if (this->station_name_text_sensor_ != nullptr)
    this->station_name_text_sensor_->publish_state(text);
#endif
}

void Si4703Component::set_station_name_status(const char *text) {
  if (!this->status_text_)
    return;
  strncpy(this->status_, text, sizeof(this->status_) - 1);
  this->status_[sizeof(this->status_) - 1] = '\0';
  this->publish_station_status_(this->status_);
}

void Si4703Component::process_rds_() {
  if (!(this->registers_[REG_STATUSRSSI] & ST_RDSR))
    return;
  // Verbose mode error levels: 0 none, 1 = 1-2 bits corrected, 2 = 3-5 corrected, 3 = uncorrectable.
  // Corrected blocks are accepted; text is only published after two identical complete reads.
  const uint16_t rc = this->registers_[REG_READCHAN];
  const uint8_t bler_b = (rc >> 14) & 0x03;
  const uint8_t bler_c = (rc >> 12) & 0x03;
  const uint8_t bler_d = (rc >> 10) & 0x03;
  if (bler_b > 1)
    return;  // block B carries group type and segment address

  const uint16_t b = this->registers_[REG_RDSB];
  const uint16_t c = this->registers_[REG_RDSC];
  const uint16_t d = this->registers_[REG_RDSD];
  const uint8_t group = b >> 12;
  const bool version_b = (b >> 11) & 0x01;
  char text[65];

  if (group == 0) {
    // Programme Service name: 4 segments x 2 characters in block D
    if (bler_d >= RDS_UNCORRECTABLE)
      return;
    const uint8_t seg = b & 0x03;
    this->ps_[seg * 2] = rds_char(d >> 8);
    this->ps_[seg * 2 + 1] = rds_char(d & 0xFF);
    this->ps_seen_ |= 1 << seg;
    if (this->ps_seen_ != 0x0F)
      return;
    this->ps_seen_ = 0;
    trim_copy(this->ps_, 8, text);
    if (text[0] != '\0' && strcmp(text, this->ps_candidate_) == 0 && strcmp(text, this->ps_published_) != 0) {
      strcpy(this->ps_published_, text);
      ESP_LOGD(TAG, "RDS station: '%s'", text);
#ifdef USE_TEXT_SENSOR
      if (this->station_name_text_sensor_ != nullptr)
        this->station_name_text_sensor_->publish_state(text);
#endif
    }
    strcpy(this->ps_candidate_, text);
  } else if (group == 2) {
    // RadioText: 2A = 16 segments x 4 chars (blocks C+D), 2B = 16 x 2 chars (block D)
    if (bler_d >= RDS_UNCORRECTABLE || (!version_b && bler_c >= RDS_UNCORRECTABLE))
      return;
    const int8_t ab = (b >> 4) & 0x01;
    if (this->rt_ab_ != -1 && ab != this->rt_ab_) {
      memset(this->rt_, 0, sizeof(this->rt_));
      this->rt_seen_ = 0;
    }
    this->rt_ab_ = ab;
    const uint8_t seg = b & 0x0F;
    const uint8_t chars[4] = {uint8_t(c >> 8), uint8_t(c & 0xFF), uint8_t(d >> 8), uint8_t(d & 0xFF)};
    const uint8_t first = version_b ? 2 : 0;
    const uint8_t width = version_b ? 2 : 4;
    bool done = false;
    for (uint8_t i = 0; i < width; i++) {
      const uint8_t pos = seg * width + i;
      const uint8_t ch = chars[first + i];
      if (ch == 0x0D) {  // end of message
        this->rt_[pos] = '\0';
        done = true;
        break;
      }
      this->rt_[pos] = rds_char(ch);
    }
    this->rt_seen_ |= 1 << seg;
    const uint16_t needed = done ? uint16_t((1u << (seg + 1)) - 1) : 0xFFFF;
    if ((this->rt_seen_ & needed) != needed)
      return;
    this->rt_seen_ = 0;
    trim_copy(this->rt_, 64, text);
    if (text[0] != '\0' && strcmp(text, this->rt_candidate_) == 0 && strcmp(text, this->rt_published_) != 0) {
      strcpy(this->rt_published_, text);
      ESP_LOGD(TAG, "RDS text: '%s'", text);
#ifdef USE_TEXT_SENSOR
      if (this->radio_text_text_sensor_ != nullptr)
        this->radio_text_text_sensor_->publish_state(text);
#endif
    }
    strcpy(this->rt_candidate_, text);
  }
}

}  // namespace esphome::si4703
