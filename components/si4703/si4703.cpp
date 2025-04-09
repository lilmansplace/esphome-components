#include "si4703.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h" // Include for esphome::delay

namespace esphome {
namespace si4703 {

static const char *const TAG = "si4703";

// Define Si4703 Register Addresses (add more as needed)
static const uint8_t SI4703_REG_DEVICEID = 0x00;
static const uint8_t SI4703_REG_CHIPID = 0x01;
static const uint8_t SI4703_REG_POWERCFG = 0x02;
static const uint8_t SI4703_REG_CHANNEL = 0x03;
static const uint8_t SI4703_REG_SYSCONFIG1 = 0x04;
static const uint8_t SI4703_REG_SYSCONFIG2 = 0x05;
static const uint8_t SI4703_REG_SYSCONFIG3 = 0x06;
static const uint8_t SI4703_REG_TEST1 = 0x07;
static const uint8_t SI4703_REG_TEST2 = 0x08;
static const uint8_t SI4703_REG_BOOTCONFIG = 0x09;
static const uint8_t SI4703_REG_STATUSRSSI = 0x0A;
static const uint8_t SI4703_REG_READCHAN = 0x0B;
static const uint8_t SI4703_REG_RDSA = 0x0C;
static const uint8_t SI4703_REG_RDSB = 0x0D;
static const uint8_t SI4703_REG_RDSC = 0x0E;
static const uint8_t SI4703_REG_RDSD = 0x0F;

// Define bits within registers
static const uint16_t SI4703_BIT_DISABLE = 1 << 15; // POWERCFG Disable
static const uint16_t SI4703_BIT_ENABLE = 1 << 0;   // POWERCFG Enable
static const uint16_t SI4703_BIT_TUNE = 1 << 15;     // CHANNEL Tune bit
static const uint16_t SI4703_BIT_STC = 1 << 14;      // STATUSRSSI Seek/Tune Complete
static const uint16_t SI4703_BIT_RDS = 1 << 11;      // SYSCONFIG1 RDS Enable
static const uint16_t SI4703_BIT_DE_50 = 1 << 10;    // SYSCONFIG1 De-emphasis 50us
static const uint16_t SI4703_BIT_DMUTE = 1 << 14;    // POWERCFG Disable Mute
static const uint16_t SI4703_BIT_MONO = 1 << 13;     // POWERCFG Force Mono
static const uint16_t SI4703_BIT_SEEK = 1 << 8;      // POWERCFG Seek Enable
static const uint16_t SI4703_BIT_SEEKUP = 1 << 9;    // POWERCFG Seek Direction Up


float Si4703Component::get_setup_priority() const { return setup_priority::DATA; }

void Si4703Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Si4703...");
  
  // Perform hardware reset if reset pin is configured
  if (this->reset_pin_ != nullptr) {
    this->reset_device_();
  } else {
    ESP_LOGW(TAG, "No reset pin configured. The Si4703 may not initialize properly.");
  }

  // Basic initialization:
  // 1. Read initial registers
  // 2. Power up the chip
  // 3. Set initial configuration (volume, band, etc.)

  if (!this->read_registers_()) {
    ESP_LOGE(TAG, "Failed to read initial registers");
    this->mark_failed();
    return;
  }

  // Power on
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DISABLE; // Clear disable bit
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_ENABLE;   // Set enable bit
  // Add other power-on settings if needed
  if (!this->write_registers_()) {
     ESP_LOGE(TAG, "Failed to power on Si4703");
     this->mark_failed();
     return;
  }

  // Wait for power-up (refer to datasheet, usually ~110ms for oscillator)
  esphome::delay(110); // Use esphome::delay instead of delay

  // Read status after power up
  if (!this->read_registers_()) {
      ESP_LOGW(TAG, "Failed to read registers after power up");
      // Might not be fatal, continue setup
  }

  // Set initial configurations (Example: Volume, Band, De-emphasis)
  this->registers_[SI4703_REG_SYSCONFIG1] |= (1 << 11); // Set RDS enable
  this->registers_[SI4703_REG_SYSCONFIG1] |= (1 << 10); // Set DE (De-emphasis) - 50us for Europe/Australia
                                                   // Clear for 75us (USA)
  this->registers_[SI4703_REG_SYSCONFIG2] &= 0xFFF0;    // Clear Volume bits
  this->registers_[SI4703_REG_SYSCONFIG2] |= 1;         // Set Volume to 1 (lowest audible)
  this->current_volume_ = 1;

  if (!this->write_registers_()) {
      ESP_LOGW(TAG, "Failed to set initial configuration");
  }

  // Register a 5-second update callback for sensors
  if (this->frequency_sensor_ != nullptr) {
    this->set_interval("update", 5000, [this]() { this->update(); });
  }

  ESP_LOGCONFIG(TAG, "Si4703 Setup Complete.");
}

void Si4703Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Si4703 FM Tuner:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with Si4703 failed!");
  }
  
  // Log reset pin status
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  
  // Log current status if available
  ESP_LOGCONFIG(TAG, "  Current Frequency: %.1f MHz", this->current_frequency_);
  ESP_LOGCONFIG(TAG, "  Current Volume: %u", this->current_volume_);
  ESP_LOGCONFIG(TAG, "  Mono: %s", this->current_mono_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Muted: %s", this->muted_ ? "YES" : "NO");
}

// --- I2C Helper Methods ---

bool Si4703Component::read_registers_() {
  // Si4703 requires reading registers sequentially starting from 0x0A
  // then wrapping around from 0x00 up to 0x09.
  // It returns 32 bytes (16 registers, 2 bytes each, MSB first)
  uint8_t buffer[32];
  i2c::ErrorCode err = this->read(buffer, 32);
  if (err != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C read failed with error code %d", (int)err);
      return false;
  }

  // Registers are read starting from STATUSRSSI (0x0A)
  int reg_index = SI4703_REG_STATUSRSSI; // 10
  for (int i = 0; i < 12; i += 2) { // Read 0x0A to 0x0F
      this->registers_[reg_index] = (buffer[i] << 8) | buffer[i + 1];
      reg_index++;
  }
  // Wrap around to read 0x00 to 0x09
  reg_index = SI4703_REG_DEVICEID; // 0
  for (int i = 12; i < 32; i += 2) { // Read 0x00 to 0x09
      this->registers_[reg_index] = (buffer[i] << 8) | buffer[i + 1];
      reg_index++;
  }

  // Update internal variables based on the new register values
  this->update_internal_state_();

  return true;
}

bool Si4703Component::write_registers_() {
  // Si4703 requires writing registers sequentially from 0x02 to 0x07.
  uint8_t buffer[12]; // 6 registers * 2 bytes/register
  int buf_idx = 0;
  for (int i = SI4703_REG_POWERCFG; i <= SI4703_REG_TEST1; ++i) {
      buffer[buf_idx++] = this->registers_[i] >> 8;   // MSB
      buffer[buf_idx++] = this->registers_[i] & 0xFF; // LSB
  }

  if (this->write(buffer, 12) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C write failed");
      return false;
  }
  return true;
}

// --- Radio Control Method Implementations (Stubs) ---

void Si4703Component::set_frequency(float frequency) {
  ESP_LOGD(TAG, "Setting frequency to %.1f MHz", frequency);

  // Basic validation for US/Europe band
  if (frequency < 87.5 || frequency > 108.0) {
    ESP_LOGW(TAG, "Frequency %.1f MHz is outside the expected band (87.5-108.0 MHz)", frequency);
    return;
  }

  // Calculate channel value assuming US/Europe band (starts 87.5MHz) and 100kHz spacing
  // Channel spacing = 0.1 MHz (100kHz)
  // Bottom of band = 87.5 MHz
  uint16_t channel = static_cast<uint16_t>((frequency - 87.5f) / 0.1f);

  // Read current registers
  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before setting frequency");
    // Attempt to continue with cached values, might fail
  }

  // Update CHANNEL register with new channel and set TUNE bit
  this->registers_[SI4703_REG_CHANNEL] &= 0xFE00; // Clear channel bits (lower 10 bits)
  this->registers_[SI4703_REG_CHANNEL] |= channel; // Set new channel
  this->registers_[SI4703_REG_CHANNEL] |= SI4703_BIT_TUNE; // Set TUNE bit

  // Write registers to start tuning
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to start tuning");
    return;
  }

  // Wait for the TUNE operation to complete by polling the STC bit
  ESP_LOGV(TAG, "Waiting for tune complete (STC bit)...");
  int attempts = 0;
  const int max_attempts = 100; // Timeout after ~1 second (100 * 10ms)
  while (attempts < max_attempts) {
    esphome::delay(10); // Use esphome::delay instead of delay
    if (!this->read_registers_()) {
        ESP_LOGW(TAG, "Failed to read registers while waiting for STC");
        // Continue waiting, maybe the next read will work
    } else {
        if ((this->registers_[SI4703_REG_STATUSRSSI] & SI4703_BIT_STC) != 0) {
            ESP_LOGV(TAG, "Tune complete (STC bit is set)");
            break; // Tune complete
        }
    }
    attempts++;
  }

  if (attempts == max_attempts) {
      ESP_LOGW(TAG, "Timeout waiting for tune completion (STC bit)");
      // Try to clear TUNE bit anyway
  }

  // Clear the TUNE bit (must be done after STC is set)
  this->registers_[SI4703_REG_CHANNEL] &= ~SI4703_BIT_TUNE;

  // Write registers again to clear the TUNE bit
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to clear TUNE bit");
  }

  // Update internal tracking of frequency even if read fails
  this->current_frequency_ = frequency;
  
  // Read final status to update internal state
  if (!this->read_registers_()) {
      ESP_LOGW(TAG, "Failed to read final registers after tuning");
      
      // Force update of the frequency sensor
      if (this->frequency_sensor_ != nullptr) {
        float frequency_hz = frequency * 1000000.0f;
        this->frequency_sensor_->publish_state(frequency_hz);
      }
  }
}

// Add a helper function to update internal state from registers_
void Si4703Component::update_internal_state_() {
    // Update frequency based on READCHAN
    uint16_t current_channel = this->registers_[SI4703_REG_READCHAN] & 0x03FF;
    // Assuming 100kHz spacing and 87.5MHz base
    this->current_frequency_ = 87.5f + (static_cast<float>(current_channel) * 0.1f);

    // Update volume based on SYSCONFIG2
    this->current_volume_ = this->registers_[SI4703_REG_SYSCONFIG2] & 0x000F;

    // Update mono status based on POWERCFG
    this->current_mono_ = (this->registers_[SI4703_REG_POWERCFG] & SI4703_BIT_MONO) != 0;

    // Update mute status based on POWERCFG - DMUTE means "disable mute" when set
    this->muted_ = (this->registers_[SI4703_REG_POWERCFG] & SI4703_BIT_DMUTE) == 0;

    ESP_LOGD(TAG, "Internal state updated: Freq=%.1fMHz, Vol=%d, Mono=%s, Muted=%s",
             this->current_frequency_, this->current_volume_,
             this->current_mono_ ? "true" : "false",
             this->muted_ ? "true" : "false");

    // Update frequency sensor if available
    if (this->frequency_sensor_ != nullptr) {
        float frequency_hz = this->current_frequency_ * 1000000.0f; // Convert MHz to Hz
        this->frequency_sensor_->publish_state(frequency_hz);
    }
}

void Si4703Component::set_volume(uint8_t volume) {
  ESP_LOGD(TAG, "Setting volume to %u", volume);
  if (volume > 15) volume = 15; // Clamp volume
  this->current_volume_ = volume;

  // Read current registers to preserve other settings
  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before setting volume");
    // Continue anyway, hoping the cached register value is okay
  }

  // Update volume bits in SYSCONFIG2 register
  this->registers_[SI4703_REG_SYSCONFIG2] &= 0xFFF0; // Clear volume bits (lower 4 bits)
  this->registers_[SI4703_REG_SYSCONFIG2] |= volume; // Set new volume

  // Check mute status - if volume > 0, ensure DMUTE is off
  if (volume > 0 && this->muted_) {
    this->registers_[SI4703_REG_POWERCFG] &= ~(1 << 14); // Clear DMUTE bit
    this->muted_ = false;
  } else if (volume == 0) {
    this->registers_[SI4703_REG_POWERCFG] |= (1 << 14); // Set DMUTE bit
    this->muted_ = true;
  }

  // Write the updated registers back to the chip
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers for setting volume");
  }
}

void Si4703Component::seek_up() {
  ESP_LOGD(TAG, "Seeking up...");

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before seeking up");
  }

  // Set SEEKUP bit (direction up)
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_SEEKUP;
  // Set SEEK bit to start operation
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_SEEK;

  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to start seek up");
    return;
  }

  // Wait for the SEEK operation to complete by polling the STC bit
  ESP_LOGV(TAG, "Waiting for seek complete (STC bit)...");
  int attempts = 0;
  const int max_attempts = 500; // Seek can take longer, ~5 seconds timeout
  while (attempts < max_attempts) {
    esphome::delay(10); // Use esphome::delay instead of delay
    if (!this->read_registers_()) {
        ESP_LOGW(TAG, "Failed to read registers while waiting for STC (seek)");
    } else {
        if ((this->registers_[SI4703_REG_STATUSRSSI] & SI4703_BIT_STC) != 0) {
            ESP_LOGV(TAG, "Seek complete (STC bit is set)");
            break; // Seek complete
        }
    }
    attempts++;
  }

  if (attempts == max_attempts) {
      ESP_LOGW(TAG, "Timeout waiting for seek completion (STC bit)");
      // Still attempt to clear seek bit
  }

  // Read registers to update internal state with the new frequency found
  // The update_internal_state_() is called inside read_registers_()
  if (!this->read_registers_()){
      ESP_LOGW(TAG, "Failed to read registers after seek completion");
  }

  // Clear the SEEK bit (must be done after STC is set)
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_SEEK;

  // Write registers again to clear the SEEK bit
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to clear SEEK bit after seek up");
  }

  // TODO: Maybe check the SF/BL bit (Seek Fail/Band Limit) in STATUSRSSI?
}

void Si4703Component::seek_down() {
  ESP_LOGD(TAG, "Seeking down...");

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before seeking down");
  }

  // Clear SEEKUP bit (direction down)
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_SEEKUP;
  // Set SEEK bit to start operation
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_SEEK;

  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to start seek down");
    return;
  }

  // Wait for the SEEK operation to complete (polling STC bit)
  ESP_LOGV(TAG, "Waiting for seek complete (STC bit)...");
  int attempts = 0;
  const int max_attempts = 500; // ~5 seconds timeout
  while (attempts < max_attempts) {
    esphome::delay(10); // Use esphome::delay instead of delay
    if (!this->read_registers_()) {
        ESP_LOGW(TAG, "Failed to read registers while waiting for STC (seek)");
    } else {
        if ((this->registers_[SI4703_REG_STATUSRSSI] & SI4703_BIT_STC) != 0) {
            ESP_LOGV(TAG, "Seek complete (STC bit is set)");
            break; // Seek complete
        }
    }
    attempts++;
  }

   if (attempts == max_attempts) {
      ESP_LOGW(TAG, "Timeout waiting for seek completion (STC bit)");
      // Still attempt to clear seek bit
  }

  // Read registers to update internal state
  if (!this->read_registers_()){
      ESP_LOGW(TAG, "Failed to read registers after seek completion");
  }

  // Clear the SEEK bit
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_SEEK;

  // Write registers again to clear the SEEK bit
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers to clear SEEK bit after seek down");
  }
}

void Si4703Component::set_mono(bool mono) {
  ESP_LOGD(TAG, "Setting mono to %s", mono ? "ON" : "OFF");
  this->current_mono_ = mono;

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before setting mono");
  }

  if (mono) {
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_MONO; // Set MONO bit
  } else {
    this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_MONO; // Clear MONO bit
  }

  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers for setting mono");
  }
}

void Si4703Component::mute() {
  ESP_LOGD(TAG, "Muting audio");
  this->muted_ = true;

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before muting");
  }

  // Set DMUTE bit in POWERCFG register
  this->registers_[SI4703_REG_POWERCFG] |= (1 << 14);

  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers for muting");
  }
}

void Si4703Component::unmute() {
  ESP_LOGD(TAG, "Unmuting audio");
  this->muted_ = false;

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before unmuting");
  }

  // Clear DMUTE bit in POWERCFG register
  this->registers_[SI4703_REG_POWERCFG] &= ~(1 << 14);

  // If volume was previously 0, set it to a minimum audible level (e.g., 1)
  if (this->current_volume_ == 0) {
    this->set_volume(1);
    // Note: set_volume already writes registers and handles DMUTE
    // So we don't need to call write_registers_() again here if volume was 0.
  } else {
      // Only write registers if volume was not 0 (otherwise set_volume handles it)
      if (!this->write_registers_()) {
        ESP_LOGE(TAG, "Failed to write registers for unmuting");
      }
  }
}

// Add this method to implement the reset sequence
void Si4703Component::reset_device_() {
  ESP_LOGD(TAG, "Performing hardware reset sequence for Si4703");
  
  // Configure reset pin as output
  this->reset_pin_->setup();
  
  // Si4703 specific reset sequence
  // 1. Ensure reset is HIGH initially
  this->reset_pin_->digital_write(true);
  esphome::delay(10);
  
  // 2. Pull reset LOW for at least 1ms
  this->reset_pin_->digital_write(false);
  esphome::delay(50); // 50ms for more reliable reset
  
  // 3. Release reset (HIGH)
  this->reset_pin_->digital_write(true);
  
  // 4. Critical delay for 2-wire interface initialization
  // Per datasheet: wait at least 1ms for oscillator stabilization
  // In practice, 500ms works much better for reliability
  esphome::delay(500);
  
  ESP_LOGD(TAG, "Hardware reset completed, Si4703 should now be ready for I2C communication");
}

// Add the update method
void Si4703Component::update() {
  // Update the frequency sensor
  if (this->frequency_sensor_ != nullptr) {
    // Convert from MHz to Hz for the sensor
    float frequency_hz = this->current_frequency_ * 1000000.0f;
    ESP_LOGD(TAG, "Publishing frequency: %.1f MHz (%.0f Hz)", this->current_frequency_, frequency_hz);
    this->frequency_sensor_->publish_state(frequency_hz);
  }
}

} // namespace si4703
} // namespace esphome 