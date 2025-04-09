#include "si4703_media_player.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace si4703 {

static const char *const TAG = "si4703.media_player";

void Si4703MediaPlayer::setup() {
  // Initialization if needed, but most setup is in the parent Si4703Component
  ESP_LOGCONFIG(TAG, "Setting up Si4703 Media Player '%s'...", this->name_.c_str());
  
  // Initialize with PLAYING state since the radio is on by default
  this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
  
  // Set initial volume value
  if (this->parent_ != nullptr) {
    this->volume = this->parent_->get_volume() / 15.0f; // Scale from 0-15 to 0.0-1.0
  }
  
  // Publish initial state so Home Assistant knows we're active
  this->publish_state();
}

void Si4703MediaPlayer::dump_config() {
  ESP_LOGCONFIG(TAG, "Si4703 Media Player:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  // Optionally log specific media player settings here
}

media_player::MediaPlayerTraits Si4703MediaPlayer::get_traits() {
  auto traits = media_player::MediaPlayerTraits();
  // ESPHome media player only supports pause in traits
  traits.set_supports_pause(true);
  return traits;
}

void Si4703MediaPlayer::control(const media_player::MediaPlayerCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent Si4703Component not set!");
    return;
  }

  if (call.get_command().has_value()) {
    switch (call.get_command().value()) {
      case media_player::MEDIA_PLAYER_COMMAND_PLAY:
        this->parent_->unmute(); // Or power on if applicable
        this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
        break;
      case media_player::MEDIA_PLAYER_COMMAND_PAUSE:
        this->parent_->mute(); // Or power off if applicable
        this->state = media_player::MEDIA_PLAYER_STATE_PAUSED;
        break;
      case media_player::MEDIA_PLAYER_COMMAND_MUTE:
        this->parent_->mute();
        // Don't try to set is_muted directly as it's a method
        break;
      case media_player::MEDIA_PLAYER_COMMAND_UNMUTE:
        this->parent_->unmute();
        // Don't try to set is_muted directly as it's a method
        break;
      case media_player::MEDIA_PLAYER_COMMAND_VOLUME_UP:
        // Optional: Implement gradual volume up
        // this->parent_->set_volume(this->parent_->current_volume_ + 1);
        break;
      case media_player::MEDIA_PLAYER_COMMAND_VOLUME_DOWN:
        // Optional: Implement gradual volume down
        // this->parent_->set_volume(this->parent_->current_volume_ - 1);
        break;
      // Handle seeking as up/down operations (no specific next/prev track commands)
      default:
        // Special handling for custom commands
        if (call.get_command().value() == 76) { // Custom command for NEXT_TRACK
          this->parent_->seek_up();
          this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
        } else if (call.get_command().value() == 77) { // Custom command for PREV_TRACK
          this->parent_->seek_down();
          this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
        } else {
          ESP_LOGW(TAG, "Unsupported media command: %d", call.get_command().value());
        }
        break;
    }
  }

  if (call.get_volume().has_value()) {
    float volume_level = call.get_volume().value();
    uint8_t si4703_volume = static_cast<uint8_t>(volume_level * 15.0f); // Scale 0.0-1.0 to 0-15
    this->parent_->set_volume(si4703_volume);
    this->volume = volume_level; // Update internal state
    // Don't try to set is_muted directly as it's a method
  }

  if (call.get_media_url().has_value()) {
    // Use media_url to set frequency (e.g., "101.1")
    std::string freq_str = call.get_media_url().value();
    ESP_LOGD(TAG, "Received frequency URL: %s", freq_str.c_str());
    float freq = atof(freq_str.c_str()); // Convert string to float
    if (freq >= 87.5 && freq <= 108.0) { // Basic validation
        this->parent_->set_frequency(freq);
        this->state = media_player::MEDIA_PLAYER_STATE_PLAYING; // Assume tuning starts playback
    } else {
        ESP_LOGW(TAG, "Invalid frequency received: %s", freq_str.c_str());
    }
  }

  // After other operations, update frequency information
  if (this->parent_ != nullptr) {
    char freq_str[16];
    sprintf(freq_str, "%.1f MHz", this->parent_->get_frequency());
    
    // Set station frequency in a way that will show in Home Assistant
    this->source = freq_str;
    
    // If the station is properly tuned, consider it playing
    this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
  }

  this->publish_state(); // Update Home Assistant with the new state
}

} // namespace si4703
} // namespace esphome 