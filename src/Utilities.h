#pragma once

static const float pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062f;
static const float pi_2 = pi * 2.f;
static const float half_pi = pi / 2.f;
static const float ToRad = pi / 180.f;
static const float ToDeg = 180.f / pi;

typedef Eigen::Array3f Color; // Like a Vector3f, but allows component-wise multiplication

// returns the euler angles of a given quaternion
static Eigen::Vector3f QuatToEuler(Eigen::Quaternionf q)
{
  return q.toRotationMatrix().eulerAngles(0, 1, 2) * ToDeg;
};

// returns the quaternion representation of a given set of euler angles
static Eigen::Quaternionf EulerToQuat(Eigen::Vector3f e)
{
  Eigen::Quaternionf rot =
      Eigen::AngleAxisf(e[0] * ToRad, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(e[1] * ToRad, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(e[2] * ToRad, Eigen::Vector3f::UnitZ());
  rot.normalize();
  return rot;
}

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
static std::random_device device;
static std::mt19937_64 RNGen(device());
static std::uniform_real_distribution<> myrandom(0.0, 1.0);

// returns a random float between 0 and 1
static float randf()
{
  return (float)(myrandom(RNGen));
}

// array of key codes
#include <unordered_map>
static std::unordered_map<char*, int> keyLookup{
  { "Backspace", 0x08 },
  { "Tab", 0x09 },
  { "Clear", 0x0C },
  { "Enter", 0x0D },
  { "Shift", 0x10 },
  { "Control", 0x11 },
  { "Alt", 0x12 },
  { "Pause", 0x13 },
  { "Capslock", 0x14 },
  { "Kana", 0x15 },
  { "Hanguel", 0x15 },
  { "Hangul", 0x15 },
  { "Junja", 0x17 },
  { "Final", 0x18 },
  { "Hanja", 0x19 },
  { "Kanji", 0x19 },
  { "Esc", 0x1B },
  { "Convert", 0x1C },
  { "NonConvert", 0x1D },
  { "Accept", 0x1E },
  { "ModeChange", 0x1F },
  { "Space", 0x20 },
  { "PageUp", 0x21 },
  { "PageDown", 0x22 },
  { "End", 0x23 },
  { "Home", 0x24 },
  { "LeftArrow", 0x25 },
  { "UpArrow", 0x26 },
  { "RightArrow", 0x27 },
  { "DownArrow", 0x28 },
  { "Select", 0x29 },
  { "Print", 0x2A },
  { "Execute", 0x2B },
  { "PrintScreen", 0x2C },
  { "Insert", 0x2D },
  { "Delete", 0x2E },
  { "Help", 0x2F },
  { "Alphanumeric_0", 0x30 },
  { "Alphanumeric_1", 0x31 },
  { "Alphanumeric_2", 0x32 },
  { "Alphanumeric_3", 0x33 },
  { "Alphanumeric_4", 0x34 },
  { "Alphanumeric_5", 0x35 },
  { "Alphanumeric_6", 0x36 },
  { "Alphanumeric_7", 0x37 },
  { "Alphanumeric_8", 0x38 },
  { "Alphanumeric_9", 0x39 },
  { "A", 0x41 },
  { "B", 0x42 },
  { "C", 0x43 },
  { "D", 0x44 },
  { "E", 0x45 },
  { "F", 0x46 },
  { "G", 0x47 },
  { "H", 0x48 },
  { "I", 0x49 },
  { "J", 0x4A },
  { "K", 0x4B },
  { "L", 0x4C },
  { "M", 0x4D },
  { "N", 0x4E },
  { "O", 0x4F },
  { "P", 0x50 },
  { "Q", 0x51 },
  { "R", 0x52 },
  { "S", 0x53 },
  { "T", 0x54 },
  { "U", 0x55 },
  { "V", 0x56 },
  { "W", 0x57 },
  { "X", 0x58 },
  { "Y", 0x59 },
  { "Z", 0x5A },
  { "LeftWin", 0x5B },
  { "RightWin", 0x5C },
  { "Apps", 0x5D },
  { "Sleep", 0x5F },
  { "Numpad_0", 0x60 },
  { "Numpad_1", 0x61 },
  { "Numpad_2", 0x62 },
  { "Numpad_3", 0x63 },
  { "Numpad_4", 0x64 },
  { "Numpad_5", 0x65 },
  { "Numpad_6", 0x66 },
  { "Numpad_7", 0x67 },
  { "Numpad_8", 0x68 },
  { "Numpad_9", 0x69 },
  { "Multiply", 0x6A },
  { "Add", 0x6B },
  { "Separator", 0x6C },
  { "Subtract", 0x6D },
  { "Decimal", 0x6E },
  { "Divide", 0x6F },
  { "F1", 0x70 },
  { "F2", 0x71 },
  { "F3", 0x72 },
  { "F4", 0x73 },
  { "F5", 0x74 },
  { "F6", 0x75 },
  { "F7", 0x76 },
  { "F8", 0x77 },
  { "F9", 0x78 },
  { "F10", 0x79 },
  { "F11", 0x7A },
  { "F12", 0x7B },
  { "F13", 0x7C },
  { "F14", 0x7D },
  { "F15", 0x7E },
  { "F16", 0x7F },
  { "F17", 0x80 },
  { "F18", 0x81 },
  { "F19", 0x82 },
  { "F20", 0x83 },
  { "F21", 0x84 },
  { "F22", 0x85 },
  { "F23", 0x86 },
  { "F24", 0x87 },
  { "Numlock", 0x90 },
  { "ScrollLock", 0x91 },
  { "LeftShift", 0xA0 },
  { "RightShift", 0xA1 },
  { "LeftCtrl", 0xA2 },
  { "RightCtrl", 0xA3 },
  { "LeftAlt", 0xA4 },
  { "RightAlt", 0xA5 },
  { "BrowserBack", 0xA6 },
  { "BrowserForward", 0xA7 },
  { "BrowserRefresh", 0xA8 },
  { "BrowserStop", 0xA9 },
  { "BrowserSearch", 0xAA },
  { "BrowserFavorites", 0xAB },
  { "BrowserHome", 0xAC },
  { "VolumeMute", 0xAD },
  { "VolumeDown", 0xAE },
  { "VolumeUp", 0xAF },
  { "MediaNextTrack", 0xB0 },
  { "MediaPrevTrack", 0xB1 },
  { "MediaStop", 0xB2 },
  { "MediaPlayPause", 0xB3 },
  { "LaunchMail", 0xB4 },
  { "LaunchMedia", 0xB5 },
  { "LaunchApp1", 0xB6 },
  { "LaunchApp2", 0xB7 },
  { "OEM_1", 0xBA },
  { "OEM_Plus", 0xBB },
  { "OEM_Comma", 0xBC },
  { "OEM_Minus", 0xBD },
  { "OEM_Period", 0xBE },
  { "OEM_2", 0xBF },
  { "OEM_3", 0xC0 },
  { "OEM_4", 0xDB },
  { "OEM_5", 0xDC },
  { "OEM_6", 0xDD },
  { "OEM_7", 0xDE },
  { "OEM_8", 0xDF },
  { "OEM_102", 0xE2 },
  { "ProcessKey", 0xE5 },
  { "Packet", 0xE7 },
  { "Attn", 0xF6 },
  { "CrSel", 0xF7 },
  { "ExSel", 0xF8 },
  { "EREOF", 0xF9 },
  { "Play", 0xFA },
  { "Zoom", 0xFB },
  { "NONAME", 0xFC },
  { "PA1", 0xFD },
  { "OEM_Clear", 0xFE },
  { "None", 0x00 }
};