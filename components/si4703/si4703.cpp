#include "si4703.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace si4703 {

static const char *const TAG = "si4703";

// Register indexes
static const uint8_t REG_DEVICEID = 0x00;
static const uint8_t REG_CHIPID = 0x01;
static const uint8_t REG_POWERCFG = 0x02;
static const uint8_t REG_CHANNEL = 0x03;
static const uint8_t REG_SYSCONFIG1 = 0x04;
static const uint8_t REG_SYSCONFIG2 = 0x05;
static const uint8_t REG_SYSCONFIG3 = 0x06;
static const uint8_t REG_TEST1 = 0x07;
static const uint8_t REG_STATUSRSSI = 0x0A;
static const uint8_t REG_READCHAN = 0x0B;
static const uint8_t REG_RDSA = 0x0C;
static const uint8_t REG_RDSB = 0x0D;
static const uint8_t REG_RDSC = 0x0E;
static const uint8_t REG_RDSD = 0x0F;

static const uint16_t EXPECTED_DEVICEID = 0x1242;

// POWERCFG (0x02)
static const uint16_t PWR_DSMUTE = 1 << 15;
static const uint16_t PWR_DMUTE = 1 << 14;
static const uint16_t PWR_MONO = 1 << 13;
static const uint16_t PWR_SKMODE = 1 << 10;  // 1 = stop at band limit, 0 = wrap
static const uint16_t PWR_SEEKUP = 1 << 9;
static const uint16_t PWR_SEEK = 1 << 8;
static const uint16_t PWR_DISABLE = 1 << 6;
static const uint16_t PWR_ENABLE = 1 << 0;
// CHANNEL (0x03)
static const uint16_t CHAN_TUNE = 1 << 15;
// SYSCONFIG1 (0x04)
static const uint16_t SC1_RDS = 1 << 12;
static const uint16_t SC1_DE = 1 << 11;  // 1 = 50us, 0 = 75us
// SYSCONFIG2 (0x05): SEEKTH[15:8] BAND[7:6] SPACE[5:4] VOLUME[3:0]
static const uint16_t SC2_SPACE_100K = 0x01 << 4;
// SYSCONFIG3 (0x06): SKSNR[7:4] SKCNT[3:0]
static const uint16_t SC3_SEEK_DEFAULTS = (4 << 4) | 8;
// TEST1 (0x07)
static const uint16_t TEST1_XOSCEN = 0x8100;  // AN230: crystal on (SparkFun board has a 32.768kHz crystal)
// STATUSRSSI (0x0A)
static const uint16_t ST_RDSR = 1 << 15;
static const uint16_t ST_STC = 1 << 14;
static const uint16_t ST_SFBL = 1 << 13;
static const uint16_t ST_ST = 1 << 8;
// READCHAN (0x0B) block error bits for B, C, D
static const uint16_t RC_BLER_MASK = 0xFC00;

static const float BAND_BOTTOM = 87.5f;   // BAND = 00 : 87.5 - 108 MHz
static const float CHANNEL_STEP = 0.1f;   // SPACE = 01: 100 kHz
static const uint32_t POLL_INTERVAL_MS = 40;
static const uint32_t RSSI_PUBLISH_MS = 2000;
static const uint32_t SEEK_TIMEOUT_MS = 10000;
static const uint8_t MAX_INIT_ATTEMPTS = 5;

// Must run before the I2C bus (priority BUS) so we can drive SDIO as a plain GPIO during reset.
float Si4703Component::get_setup_priority() const { return setup_priority::BUS + 1.0f; }

void Si4703Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Si4703...");
  this->hardware_reset_();
  // The I2C bus is initialised after us; finish chip init once everything is set up.
  this->set_timeout("init", 50, [this]() { this->init_stage1_(); });
}

void Si4703Component::hardware_reset_() {
  // Si4702/03 latch the bus mode on the rising edge of RST.
  // 2-wire (I2C) mode needs SEN high (SparkFun board pulls SEN up) and SDIO LOW at that moment.
  // SparkFun's board also pulls SDIO up, so we must actively hold it low.
  this->reset_pin_->setup();
  this->reset_pin_->digital_write(false);
  if (this->sdio_pin_ != nullptr) {
    this->sdio_pin_->setup();
    this->sdio_pin_->pin_mode(gpio::FLAG_OUTPUT);
    this->sdio_pin_->digital_write(false);
  } else {
    ESP_LOGW(TAG, "No sdio_pin configured - relying on SDIO being low at reset (may land in 3-wire mode)");
  }
  delay(10);
  this->reset_pin_->digital_write(true);  // latch 2-wire mode
  delay(2);
  if (this->sdio_pin_ != nullptr) {
    this->sdio_pin_->pin_mode(gpio::FLAG_INPUT);  // release SDIO for the I2C driver
  }
  ESP_LOGD(TAG, "Hardware reset done (SDIO held low while RST released)");
}

void Si4703Component::init_stage1_() {
  this->init_attempts_++;
  if (!this->read_all_() || this->registers_[REG_DEVICEID] != EXPECTED_DEVICEID) {
    ESP_LOGE(TAG, "Si4703 not responding correctly (attempt %u/%u): DEVICEID=0x%04X CHIPID=0x%04X (expected DEVICEID 0x1242)",
             this->init_attempts_, MAX_INIT_ATTEMPTS, this->registers_[REG_DEVICEID], this->registers_[REG_CHIPID]);
    if (this->init_attempts_ >= MAX_INIT_ATTEMPTS) {
      ESP_LOGE(TAG, "Giving up. Check: SEN not grounded (must be high), SDIO/SCLK/RST wiring, 3.3V supply.");
      this->mark_failed();
      return;
    }
    // Just retry the read. Do NOT pulse RST here: SDIO idles high now, so a reset would latch 3-wire mode.
    this->set_timeout("init", 1000, [this]() { this->init_stage1_(); });
    return;
  }
  ESP_LOGI(TAG, "Found Si4703: DEVICEID=0x%04X CHIPID=0x%04X", this->registers_[REG_DEVICEID],
           this->registers_[REG_CHIPID]);

  // Start the 32.768 kHz crystal oscillator and wait >= 500 ms (AN230).
  this->registers_[REG_TEST1] = TEST1_XOSCEN;
  if (!this->write_regs_()) {
    ESP_LOGE(TAG, "Failed to enable crystal oscillator");
    this->mark_failed();
    return;
  }
  this->set_timeout("init", 600, [this]() { this->init_stage2_(); });
}

void Si4703Component::init_stage2_() {
  if (!this->read_all_()) {
    ESP_LOGE(TAG, "Read failed after oscillator start");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Oscillator check: TEST1=0x%04X", this->registers_[REG_TEST1]);

  uint16_t pwr = PWR_ENABLE;
  if (!this->muted_)
    pwr |= PWR_DMUTE;
  if (this->current_mono_)
    pwr |= PWR_MONO;
  this->registers_[REG_POWERCFG] = pwr;
  this->registers_[REG_SYSCONFIG1] = SC1_RDS | (this->de_50us_ ? SC1_DE : 0);
  this->registers_[REG_SYSCONFIG2] =
      (uint16_t(this->seek_threshold_) << 8) | SC2_SPACE_100K | (this->current_volume_ & 0x0F);
  this->registers_[REG_SYSCONFIG3] = SC3_SEEK_DEFAULTS;

  if (!this->write_regs_()) {
    ESP_LOGE(TAG, "Failed to power up Si4703");
    this->mark_failed();
    return;
  }

  this->set_timeout("init", 120, [this]() {
    this->read_all_();
    this->ready_ = true;
    ESP_LOGI(TAG, "Si4703 powered up (POWERCFG=0x%04X SYSCONFIG2=0x%04X)", this->registers_[REG_POWERCFG],
             this->registers_[REG_SYSCONFIG2]);
    this->set_frequency(this->initial_frequency_);
    this->log_registers();
  });
}

void Si4703Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Si4703 FM Tuner:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  SDIO Pin: ", this->sdio_pin_);
  ESP_LOGCONFIG(TAG, "  Band: 87.5-108 MHz, 100 kHz spacing, de-emphasis %s", this->de_50us_ ? "50us" : "75us");
  ESP_LOGCONFIG(TAG, "  Start frequency: %.1f MHz, volume %u", this->initial_frequency_, this->current_volume_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Communication with Si4703 failed!");
  }
  LOG_SENSOR("  ", "Frequency", this->frequency_sensor_);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  LOG_TEXT_SENSOR("  ", "Station Name", this->station_name_sensor_);
  LOG_TEXT_SENSOR("  ", "Radio Text", this->radio_text_sensor_);
}

void Si4703Component::loop() {
  if (!this->ready_)
    return;
  const uint32_t now = millis();
  if (now - this->last_poll_ < POLL_INTERVAL_MS)
    return;
  this->last_poll_ = now;

  if (!this->read_status_())
    return;

  if (this->seeking_) {
    if (this->registers_[REG_STATUSRSSI] & ST_STC) {
      this->finish_seek_();
    } else if (now - this->seek_started_ > SEEK_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Seek timed out");
      this->finish_seek_();
    }
    return;
  }

  this->process_rds_();

  if (now - this->last_rssi_publish_ >= RSSI_PUBLISH_MS) {
    this->last_rssi_publish_ = now;
    if (this->rssi_sensor_ != nullptr)
      this->rssi_sensor_->publish_state(this->rssi_);
  }
}

// ---------------------------------------------------------------- I2C helpers

bool Si4703Component::read_all_() {
  uint8_t buf[32];
  if (this->read(buf, sizeof(buf)) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C read (32 bytes) failed");
    return false;
  }
  // Reads start at 0x0A and wrap: 0x0A..0x0F, then 0x00..0x09
  uint8_t reg = REG_STATUSRSSI;
  for (int i = 0; i < 32; i += 2) {
    this->registers_[reg] = (uint16_t(buf[i]) << 8) | buf[i + 1];
    reg = (reg + 1) & 0x0F;
  }
  this->rssi_ = this->registers_[REG_STATUSRSSI] & 0xFF;
  this->stereo_ = (this->registers_[REG_STATUSRSSI] & ST_ST) != 0;
  return true;
}

bool Si4703Component::read_status_() {
  uint8_t buf[12];
  if (this->read(buf, sizeof(buf)) != i2c::ERROR_OK) {
    ESP_LOGV(TAG, "I2C status read failed");
    return false;
  }
  for (int i = 0; i < 6; i++) {
    this->registers_[REG_STATUSRSSI + i] = (uint16_t(buf[2 * i]) << 8) | buf[2 * i + 1];
  }
  this->rssi_ = this->registers_[REG_STATUSRSSI] & 0xFF;
  this->stereo_ = (this->registers_[REG_STATUSRSSI] & ST_ST) != 0;
  return true;
}

bool Si4703Component::write_regs_() {
  // Writes always start at 0x02 and run upward; we write 0x02..0x07.
  uint8_t buf[12];
  int idx = 0;
  for (uint8_t reg = REG_POWERCFG; reg <= REG_TEST1; reg++) {
    buf[idx++] = this->registers_[reg] >> 8;
    buf[idx++] = this->registers_[reg] & 0xFF;
  }
  if (this->write(buf, sizeof(buf)) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C write failed");
    return false;
  }
  return true;
}

bool Si4703Component::wait_stc_(bool want_set, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (this->read_status_()) {
      bool stc = (this->registers_[REG_STATUSRSSI] & ST_STC) != 0;
      if (stc == want_set)
        return true;
    }
    delay(5);
  }
  return false;
}

void Si4703Component::refresh_frequency_() {
  uint16_t chan = this->registers_[REG_READCHAN] & 0x03FF;
  this->current_frequency_ = BAND_BOTTOM + chan * CHANNEL_STEP;
  // Round to one decimal to avoid 96.29999
  this->current_frequency_ = std::round(this->current_frequency_ * 10.0f) / 10.0f;
  if (this->frequency_sensor_ != nullptr)
    this->frequency_sensor_->publish_state(this->current_frequency_);
  if (this->rssi_sensor_ != nullptr)
    this->rssi_sensor_->publish_state(this->rssi_);
}

void Si4703Component::log_registers() {
  if (!this->read_all_()) {
    ESP_LOGE(TAG, "Register dump: read failed");
    return;
  }
  const uint16_t *r = this->registers_;
  ESP_LOGI(TAG, "Regs 00-07: %04X %04X %04X %04X %04X %04X %04X %04X", r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
  ESP_LOGI(TAG, "Regs 08-0F: %04X %04X %04X %04X %04X %04X %04X %04X", r[8], r[9], r[10], r[11], r[12], r[13], r[14],
           r[15]);
  ESP_LOGI(TAG, "  POWERCFG=%04X (ENABLE=%d DISABLE=%d DMUTE=%d) TEST1=%04X (XOSCEN=%d)", r[REG_POWERCFG],
           (r[REG_POWERCFG] & PWR_ENABLE) != 0, (r[REG_POWERCFG] & PWR_DISABLE) != 0,
           (r[REG_POWERCFG] & PWR_DMUTE) != 0, r[REG_TEST1], (r[REG_TEST1] & 0x8000) != 0);
  ESP_LOGI(TAG, "  STATUS=%04X RSSI=%u STC=%d SF/BL=%d ST=%d READCHAN=%u", r[REG_STATUSRSSI], r[REG_STATUSRSSI] & 0xFF,
           (r[REG_STATUSRSSI] & ST_STC) != 0, (r[REG_STATUSRSSI] & ST_SFBL) != 0, (r[REG_STATUSRSSI] & ST_ST) != 0,
           r[REG_READCHAN] & 0x03FF);
}

// ---------------------------------------------------------------- control

void Si4703Component::set_frequency(float mhz) {
  if (!this->ready_) {
    this->initial_frequency_ = mhz;  // applied when init completes
    ESP_LOGD(TAG, "Not ready yet, will tune to %.1f MHz after init", mhz);
    return;
  }
  if (mhz < 87.5f)
    mhz = 87.5f;
  if (mhz > 108.0f)
    mhz = 108.0f;
  if (this->seeking_) {
    this->registers_[REG_POWERCFG] &= ~PWR_SEEK;
    this->write_regs_();
    this->seeking_ = false;
  }

  const uint16_t chan = static_cast<uint16_t>(std::lround((mhz - BAND_BOTTOM) / CHANNEL_STEP));
  ESP_LOGD(TAG, "Tuning to %.1f MHz (channel %u)", mhz, chan);

  this->registers_[REG_CHANNEL] = CHAN_TUNE | (chan & 0x03FF);
  if (!this->write_regs_())
    return;
  if (!this->wait_stc_(true, 300))
    ESP_LOGW(TAG, "Tune did not complete (STC never set)");

  this->registers_[REG_CHANNEL] &= ~CHAN_TUNE;
  this->write_regs_();
  this->wait_stc_(false, 100);

  this->clear_rds_();
  this->refresh_frequency_();
  ESP_LOGI(TAG, "Tuned to %.1f MHz, RSSI %d dBuV, %s", this->current_frequency_, this->rssi_,
           this->stereo_ ? "stereo" : "mono");
}

void Si4703Component::start_seek_(bool up) {
  if (!this->ready_ || this->seeking_)
    return;
  ESP_LOGD(TAG, "Seeking %s...", up ? "up" : "down");
  this->registers_[REG_POWERCFG] &= ~(PWR_SKMODE | PWR_SEEKUP);  // wrap at band edge
  if (up)
    this->registers_[REG_POWERCFG] |= PWR_SEEKUP;
  this->registers_[REG_POWERCFG] |= PWR_SEEK;
  if (!this->write_regs_())
    return;
  this->seeking_ = true;
  this->seek_started_ = millis();
}

void Si4703Component::finish_seek_() {
  const bool failed = (this->registers_[REG_STATUSRSSI] & ST_SFBL) != 0;
  this->registers_[REG_POWERCFG] &= ~PWR_SEEK;
  this->write_regs_();
  this->wait_stc_(false, 100);
  this->seeking_ = false;
  this->clear_rds_();
  this->refresh_frequency_();
  if (failed) {
    ESP_LOGW(TAG, "Seek found no station (stopped at %.1f MHz)", this->current_frequency_);
  } else {
    ESP_LOGI(TAG, "Seek found %.1f MHz, RSSI %d dBuV", this->current_frequency_, this->rssi_);
  }
}

void Si4703Component::set_volume(uint8_t volume) {
  if (volume > 15)
    volume = 15;
  this->current_volume_ = volume;
  if (!this->ready_)
    return;
  ESP_LOGD(TAG, "Volume %u", volume);
  this->registers_[REG_SYSCONFIG2] = (this->registers_[REG_SYSCONFIG2] & 0xFFF0) | volume;
  this->write_regs_();
}

void Si4703Component::set_mono(bool mono) {
  this->current_mono_ = mono;
  if (!this->ready_)
    return;
  if (mono)
    this->registers_[REG_POWERCFG] |= PWR_MONO;
  else
    this->registers_[REG_POWERCFG] &= ~PWR_MONO;
  this->write_regs_();
}

void Si4703Component::mute() {
  this->muted_ = true;
  if (!this->ready_)
    return;
  this->registers_[REG_POWERCFG] &= ~PWR_DMUTE;
  this->write_regs_();
}

void Si4703Component::unmute() {
  this->muted_ = false;
  if (!this->ready_)
    return;
  this->registers_[REG_POWERCFG] |= PWR_DMUTE;
  this->write_regs_();
}

// ---------------------------------------------------------------- RDS

void Si4703Component::clear_rds_() {
  memset(this->ps_, 0, sizeof(this->ps_));
  memset(this->rt_, 0, sizeof(this->rt_));
  this->ps_seen_ = 0;
  this->rt_seen_ = 0;
  this->rt_ab_ = -1;
  this->last_ps_.clear();
  this->last_rt_.clear();
  if (this->station_name_sensor_ != nullptr)
    this->station_name_sensor_->publish_state("");
  if (this->radio_text_sensor_ != nullptr)
    this->radio_text_sensor_->publish_state("");
}

static char rds_char(uint8_t c) { return (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : ' '; }

static std::string trim(const char *s) {
  std::string out(s);
  size_t end = out.find_last_not_of(' ');
  if (end == std::string::npos)
    return "";
  out.erase(end + 1);
  size_t start = out.find_first_not_of(' ');
  return out.substr(start);
}

void Si4703Component::process_rds_() {
  if (!(this->registers_[REG_STATUSRSSI] & ST_RDSR))
    return;
  if (this->registers_[REG_READCHAN] & RC_BLER_MASK)
    return;  // B, C or D block had errors - skip

  const uint16_t b = this->registers_[REG_RDSB];
  const uint16_t c = this->registers_[REG_RDSC];
  const uint16_t d = this->registers_[REG_RDSD];
  const uint8_t group = b >> 12;
  const bool version_b = (b >> 11) & 0x01;

  if (group == 0) {
    // Programme Service name: 4 segments x 2 chars, in block D
    const uint8_t seg = b & 0x03;
    this->ps_[seg * 2] = rds_char(d >> 8);
    this->ps_[seg * 2 + 1] = rds_char(d & 0xFF);
    this->ps_seen_ |= (1 << seg);
    if (this->ps_seen_ == 0x0F) {
      std::string ps = trim(this->ps_);
      if (!ps.empty() && ps != this->last_ps_) {
        this->last_ps_ = ps;
        ESP_LOGD(TAG, "RDS station: '%s'", ps.c_str());
        if (this->station_name_sensor_ != nullptr)
          this->station_name_sensor_->publish_state(ps);
      }
      this->ps_seen_ = 0;
    }
  } else if (group == 2) {
    // RadioText
    const int8_t ab = (b >> 4) & 0x01;
    if (this->rt_ab_ != -1 && ab != this->rt_ab_) {
      memset(this->rt_, 0, sizeof(this->rt_));
      this->rt_seen_ = 0;
    }
    this->rt_ab_ = ab;
    const uint8_t seg = b & 0x0F;
    bool done = false;
    if (!version_b) {
      const uint8_t chars[4] = {uint8_t(c >> 8), uint8_t(c & 0xFF), uint8_t(d >> 8), uint8_t(d & 0xFF)};
      for (int i = 0; i < 4; i++) {
        if (chars[i] == 0x0D) {
          this->rt_[seg * 4 + i] = 0;
          done = true;
          break;
        }
        this->rt_[seg * 4 + i] = rds_char(chars[i]);
      }
    } else {
      const uint8_t chars[2] = {uint8_t(d >> 8), uint8_t(d & 0xFF)};
      for (int i = 0; i < 2; i++) {
        if (chars[i] == 0x0D) {
          this->rt_[seg * 2 + i] = 0;
          done = true;
          break;
        }
        this->rt_[seg * 2 + i] = rds_char(chars[i]);
      }
    }
    this->rt_seen_ |= (1 << seg);
    // Complete when we have every segment up to the terminator, or all 16 segments
    const uint16_t needed = done ? uint16_t((1u << (seg + 1)) - 1) : 0xFFFF;
    if ((this->rt_seen_ & needed) == needed) {
      // Stop at first NUL (unfilled positions)
      std::string rt = trim(std::string(this->rt_, strnlen(this->rt_, 64)).c_str());
      if (!rt.empty() && rt != this->last_rt_) {
        this->last_rt_ = rt;
        ESP_LOGD(TAG, "RDS text: '%s'", rt.c_str());
        if (this->radio_text_sensor_ != nullptr)
          this->radio_text_sensor_->publish_state(rt);
      }
    }
  }
}

}  // namespace si4703
}  // namespace esphome
