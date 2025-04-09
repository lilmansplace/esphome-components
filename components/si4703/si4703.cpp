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

void Si4703Component::init_preferences_() {
  if (this->preferences_initialized_)
    return;
  
#ifdef USE_ESP32
  // Create namespace for this device
  char ns[16];
  sprintf(ns, "si4703_%02x", this->address_);
  
  // Open preferences with read/write mode
  if (this->esp_preferences_.begin(ns, false)) {
    this->preferences_initialized_ = true;
    ESP_LOGD(TAG, "Initialized ESP32 preferences with namespace '%s'", ns);
  } else {
    ESP_LOGW(TAG, "Failed to initialize ESP32 preferences");
  }
#else
  ESP_LOGW(TAG, "Preferences not supported on this platform");
#endif
}

void Si4703Component::load_preferences_() {
  this->init_preferences_();
  
#ifdef USE_ESP32
  if (this->preferences_initialized_) {
    // Load frequency (in tenths of MHz for better precision)
    uint16_t freq_x10 = this->esp_preferences_.getUShort("freq", 875); // Default 87.5 MHz
    this->current_frequency_ = freq_x10 / 10.0f;
    
    // Load volume (0-15)
    this->current_volume_ = this->esp_preferences_.getUChar("vol", 1); // Default volume 1
    
    // Load flags (mono and muted in one byte)
    uint8_t flags = this->esp_preferences_.getUChar("flags", 0);
    this->current_mono_ = (flags & 0x01) != 0;
    this->muted_ = (flags & 0x02) != 0;
    
    ESP_LOGD(TAG, "Loaded settings from ESP32 preferences: Freq=%.1f MHz, Vol=%d, Mono=%s, Muted=%s",
            this->current_frequency_, this->current_volume_,
            this->current_mono_ ? "yes" : "no",
            this->muted_ ? "yes" : "no");
  } else {
    ESP_LOGD(TAG, "No ESP32 preferences available, using defaults");
  }
#endif
}

void Si4703Component::save_preferences_() {
#ifdef USE_ESP32
  if (!this->preferences_initialized_)
    this->init_preferences_();
  
  if (this->preferences_initialized_) {
    // Save frequency (as tenths of MHz for better precision)
    uint16_t freq_x10 = static_cast<uint16_t>(this->current_frequency_ * 10.0f);
    this->esp_preferences_.putUShort("freq", freq_x10);
    
    // Save volume
    this->esp_preferences_.putUChar("vol", this->current_volume_);
    
    // Save flags (mono and muted in one byte)
    uint8_t flags = 0;
    if (this->current_mono_) flags |= 0x01;
    if (this->muted_) flags |= 0x02;
    this->esp_preferences_.putUChar("flags", flags);
    
    ESP_LOGD(TAG, "Saved settings to ESP32 preferences: Freq=%.1f MHz, Vol=%d, Mono=%s, Muted=%s",
            this->current_frequency_, this->current_volume_,
            this->current_mono_ ? "yes" : "no",
            this->muted_ ? "yes" : "no");
  } else {
    ESP_LOGW(TAG, "Failed to save settings to ESP32 preferences");
  }
#endif
}

void Si4703Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Si4703...");
  
  // Load saved settings
  this->load_preferences_();
  
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
  
  // Set initial configurations from loaded preferences
  if (this->current_mono_) {
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_MONO;   // Set MONO bit
  } else {
    this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_MONO;  // Clear MONO bit
  }
  
  if (!this->muted_) {
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_DMUTE;  // Set DMUTE bit to disable muting
  } else {
    this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DMUTE; // Clear DMUTE bit to enable muting
  }
  
  if (!this->write_registers_()) {
     ESP_LOGE(TAG, "Failed to power on Si4703");
     this->mark_failed();
     return;
  }

  // Wait for power-up (refer to datasheet, usually ~110ms for oscillator)
  esphome::delay(110);

  // Read status after power up
  if (!this->read_registers_()) {
      ESP_LOGW(TAG, "Failed to read registers after power up");
      // Might not be fatal, continue setup
  }

  // Set initial volume from preferences
  this->registers_[SI4703_REG_SYSCONFIG1] |= SI4703_BIT_RDS;    // Set RDS enable
  this->registers_[SI4703_REG_SYSCONFIG1] |= SI4703_BIT_DE_50;  // Set DE (De-emphasis) - 50us for Europe/Australia
  
  // Clear and set volume bits
  this->registers_[SI4703_REG_SYSCONFIG2] &= 0xFFF0;            // Clear Volume bits
  this->registers_[SI4703_REG_SYSCONFIG2] |= (this->current_volume_ & 0x0F); // Set Volume from preferences

  if (!this->write_registers_()) {
      ESP_LOGW(TAG, "Failed to set initial configuration");
  }

  // Register a 5-second update callback for sensors
  if (this->frequency_sensor_ != nullptr) {
    this->set_interval("update", 5000, [this]() { this->update(); });
  }
  
  // Set tuner to the saved frequency
  if (this->current_frequency_ >= 87.5f && this->current_frequency_ <= 108.0f) {
    ESP_LOGD(TAG, "Setting tuner to saved frequency: %.1f MHz", this->current_frequency_);
    this->set_frequency(this->current_frequency_);
  } else {
    // Default to 87.5 if saved frequency is invalid
    ESP_LOGD(TAG, "Using default frequency: 87.5 MHz");
    this->set_frequency(87.5f);
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
  uint16_t channel = static_cast<uint16_t>((frequency - 87.5f) / 0.1f);
  ESP_LOGD(TAG, "Calculated channel value: %u for frequency %.1f MHz", channel, frequency);

  // If we have a reset pin, do a soft reset to ensure the chip is in a known state
  if (this->reset_pin_ != nullptr && this->current_frequency_ == 87.5f) {
    ESP_LOGD(TAG, "Performing soft reset before first tuning operation");
    this->reset_device_();
    // Need to re-enable the device after reset
    if (!this->read_registers_()) {
      ESP_LOGW(TAG, "Failed to read registers after reset");
    }
    this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DISABLE; // Clear disable bit
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_ENABLE;   // Set enable bit
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_DMUTE;    // Set DMUTE to enable audio
    
    if (!this->write_registers_()) {
      ESP_LOGW(TAG, "Failed to write registers after reset");
    }
    esphome::delay(110); // Wait for power-up
  }

  // Read current registers
  for (int retry = 0; retry < 3; retry++) {
    if (this->read_registers_()) {
      break;
    }
    ESP_LOGW(TAG, "Failed to read registers before setting frequency (attempt %d/3)", retry + 1);
    esphome::delay(10);
    if (retry == 2) {
      ESP_LOGE(TAG, "Could not read registers, aborting frequency change");
      return;
    }
  }

  // Log the current state of relevant registers
  ESP_LOGD(TAG, "Before tuning - CHANNEL: 0x%04X, SYSCONFIG1: 0x%04X, POWERCFG: 0x%04X", 
           this->registers_[SI4703_REG_CHANNEL], 
           this->registers_[SI4703_REG_SYSCONFIG1],
           this->registers_[SI4703_REG_POWERCFG]);

  // Update CHANNEL register with new channel and set TUNE bit
  this->registers_[SI4703_REG_CHANNEL] &= 0xFE00; // Clear channel bits (lower 10 bits)
  this->registers_[SI4703_REG_CHANNEL] |= channel; // Set new channel
  this->registers_[SI4703_REG_CHANNEL] |= SI4703_BIT_TUNE; // Set TUNE bit

  ESP_LOGD(TAG, "After setting tuning bits - CHANNEL: 0x%04X", this->registers_[SI4703_REG_CHANNEL]);

  // Ensure we're powered on
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DISABLE; // Clear disable bit
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_ENABLE;   // Set enable bit 

  // Write registers to start tuning - retry up to 3 times
  bool write_success = false;
  for (int retry = 0; retry < 3; retry++) {
    if (this->write_registers_()) {
      write_success = true;
      break;
    }
    ESP_LOGW(TAG, "Failed to write tuning registers (attempt %d/3)", retry + 1);
    esphome::delay(10);
  }

  if (!write_success) {
    ESP_LOGE(TAG, "Failed to write registers to start tuning after multiple attempts");
    return;
  }

  // Wait for the TUNE operation to complete by polling the STC bit
  ESP_LOGD(TAG, "Waiting for tune complete (STC bit)...");
  int attempts = 0;
  const int max_attempts = 150; // Increase timeout to ~1.5 seconds (150 * 10ms)
  bool tune_complete = false;
  
  while (attempts < max_attempts) {
    esphome::delay(10);
    
    // Try to read registers - don't abort if it fails sometimes
    bool read_ok = this->read_registers_();
    
    if (read_ok && (this->registers_[SI4703_REG_STATUSRSSI] & SI4703_BIT_STC) != 0) {
      ESP_LOGD(TAG, "Tune complete (STC bit is set) after %d attempts", attempts);
      tune_complete = true;
      break;
    }
    
    attempts++;
    
    // Every 10 attempts (100ms), log the register state for debugging
    if (attempts % 10 == 0 && read_ok) {
      ESP_LOGD(TAG, "Waiting for STC bit... STATUSRSSI: 0x%04X, READCHAN: 0x%04X (attempt %d/%d)",
              this->registers_[SI4703_REG_STATUSRSSI], 
              this->registers_[SI4703_REG_READCHAN],
              attempts, max_attempts);
    }
  }

  if (!tune_complete) {
    ESP_LOGW(TAG, "Timeout waiting for tune completion (STC bit)");
  }

  // Log the current state after tuning
  if (this->read_registers_()) {
    ESP_LOGD(TAG, "After tuning - CHANNEL: 0x%04X, READCHAN: 0x%04X, STATUSRSSI: 0x%04X", 
            this->registers_[SI4703_REG_CHANNEL],
            this->registers_[SI4703_REG_READCHAN],
            this->registers_[SI4703_REG_STATUSRSSI]);
  }

  // Clear the TUNE bit (must be done after STC is set)
  this->registers_[SI4703_REG_CHANNEL] &= ~SI4703_BIT_TUNE;

  // Write registers again to clear the TUNE bit - retry if needed
  write_success = false;
  for (int retry = 0; retry < 3; retry++) {
    if (this->write_registers_()) {
      write_success = true;
      break;
    }
    ESP_LOGW(TAG, "Failed to clear TUNE bit (attempt %d/3)", retry + 1);
    esphome::delay(10);
  }

  if (!write_success) {
    ESP_LOGE(TAG, "Failed to clear TUNE bit after multiple attempts");
  }

  // Force update of internal tracking even if chip communication failed
  this->current_frequency_ = frequency;
  
  // Wait a moment for the radio to settle
  esphome::delay(50);
  
  // Read final status to update internal state
  if (this->read_registers_()) {
    // Double-check if tuning was successful by checking READCHAN
    uint16_t actual_channel = this->registers_[SI4703_REG_READCHAN] & 0x03FF;
    float actual_freq = 87.5f + (static_cast<float>(actual_channel) * 0.1f);
    ESP_LOGD(TAG, "Final channel value: %u (%.1f MHz), requested: %u (%.1f MHz)", 
              actual_channel, actual_freq, channel, frequency);
      
    if (actual_channel != channel) {
      ESP_LOGW(TAG, "Tuning mismatch! Requested: %.1f MHz, Actual: %.1f MHz", 
              frequency, actual_freq);
      
      // If we consistently can't tune, we might have a deeper hardware issue
      // For now, just update internal state to match the user's request
      this->current_frequency_ = frequency;
    }
  } else {
    ESP_LOGW(TAG, "Failed to read final registers after tuning");
  }
  
  // Always update the frequency sensor with what the user requested
  // This ensures the UI stays consistent even if the hardware is having issues
  if (this->frequency_sensor_ != nullptr) {
    float frequency_hz = frequency * 1000000.0f;
    this->frequency_sensor_->publish_state(frequency_hz);
  }

  // Save the settings to preferences
  this->save_preferences_();
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

  // Check mute status - if volume > 0, ensure DMUTE is SET (unmuted)
  if (volume > 0) {
    this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_DMUTE; // Set DMUTE bit to disable muting
    this->muted_ = false;
  } else if (volume == 0) {
    this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DMUTE; // Clear DMUTE bit to enable muting
    this->muted_ = true;
  }

  // Write the updated registers back to the chip
  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers for setting volume");
  }

  // Save the settings to preferences
  this->save_preferences_();
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

  // Save the settings to preferences
  this->save_preferences_();
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

  // Save the settings to preferences
  this->save_preferences_();
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

  // Save the settings to preferences
  this->save_preferences_();
}

void Si4703Component::mute() {
  ESP_LOGD(TAG, "Muting audio");
  this->muted_ = true;

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before muting");
  }

  // Clear DMUTE bit in POWERCFG register to enable muting
  this->registers_[SI4703_REG_POWERCFG] &= ~SI4703_BIT_DMUTE;

  if (!this->write_registers_()) {
    ESP_LOGE(TAG, "Failed to write registers for muting");
  }

  // Save the settings to preferences
  this->save_preferences_();
}

void Si4703Component::unmute() {
  ESP_LOGD(TAG, "Unmuting audio");
  this->muted_ = false;

  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "Failed to read registers before unmuting");
  }

  // Set DMUTE bit in POWERCFG register to disable muting
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_DMUTE;

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

  // Save the settings to preferences
  this->save_preferences_();
}

// Add this method to implement the reset sequence
void Si4703Component::reset_device_() {
  ESP_LOGD(TAG, "Performing hardware reset sequence for Si4703");
  
  // Configure reset pin as output
  this->reset_pin_->setup();
  
  // Enhanced Si4703 reset sequence
  // Power cycle sequence - the Si4703 can be very finicky
  
  // 1. Set reset pin to LOW initially
  this->reset_pin_->digital_write(false);
  esphome::delay(100); // Wait to ensure chip is in reset state
  
  // 2. Release reset (HIGH)
  this->reset_pin_->digital_write(true);
  
  // 3. Critical delay for crystal oscillator startup and 2-wire interface initialization
  // Per datasheet, at least 1ms is needed, but in practice much longer works better
  esphome::delay(500);
  
  // 4. Read initial registers (this will force I2C bus activity which helps initialize the chip)
  // We don't actually care about the result, just want the bus traffic
  if (!this->read_registers_()) {
    ESP_LOGW(TAG, "First register read after reset failed (this is sometimes normal)");
    // Wait and try again
    esphome::delay(100);
    if (this->read_registers_()) {
      ESP_LOGD(TAG, "Second register read after reset succeeded");
    } else {
      ESP_LOGW(TAG, "Second register read after reset also failed");
    }
  } else {
    ESP_LOGD(TAG, "Initial register read after reset succeeded");
  }
  
  // Initialize with basic settings
  // Zero all register values first to avoid garbage data
  for (int i = 2; i <= 7; i++) {
    this->registers_[i] = 0;
  }
  
  // Basic configuration
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_ENABLE;   // Enable power
  this->registers_[SI4703_REG_POWERCFG] |= SI4703_BIT_DMUTE;    // Disable mute
  this->registers_[SI4703_REG_SYSCONFIG1] |= SI4703_BIT_RDS;    // Enable RDS
  this->registers_[SI4703_REG_SYSCONFIG1] |= SI4703_BIT_DE_50;  // Set 50μs De-emphasis (Europe)
  this->registers_[SI4703_REG_SYSCONFIG2] |= 9;                 // Set volume to mid level (9 out of 15)
  
  // Apply settings
  if (!this->write_registers_()) {
    ESP_LOGW(TAG, "Failed to write initial settings after reset");
  }
  
  // Wait for power-up
  esphome::delay(110);
  
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