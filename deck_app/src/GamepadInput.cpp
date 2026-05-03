#include "GamepadInput.h"
#include <iostream>

GamepadInput::GamepadInput() {
    openGamepad();
}

GamepadInput::~GamepadInput() {
    if (gamepad) {
        SDL_CloseGamepad(gamepad);
    }
}

void GamepadInput::openGamepad() {
    int count = 0;
    SDL_JoystickID* joysticks = SDL_GetGamepads(&count);
    if (joysticks && count > 0) {
        gamepad = SDL_OpenGamepad(joysticks[0]);
        if (gamepad) {
            std::cout << "Opened Gamepad: " << SDL_GetGamepadName(gamepad) << std::endl;
        }
        SDL_free(joysticks);
    }
}

float GamepadInput::applyDeadzone(int16_t axis_val, int16_t deadzone) {
    if (axis_val > deadzone) {
        return (float)(axis_val - deadzone) / (32767 - deadzone);
    } else if (axis_val < -deadzone) {
        return (float)(axis_val + deadzone) / (32768 - deadzone);
    }
    return 0.0f;
}

void GamepadInput::update() {
    if (!gamepad) {
        openGamepad();
        if (!gamepad) return;
    }

    if (!SDL_GamepadConnected(gamepad)) {
        SDL_CloseGamepad(gamepad);
        gamepad = nullptr;
        left_x = 0; left_y = 0; right_trigger = 0;
        return;
    }

    int16_t lx = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX);
    int16_t ly = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTY);
    int16_t rt = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER); // 0 to 32767

    left_x = applyDeadzone(lx);
    left_y = applyDeadzone(ly);

    // Right trigger
    if (rt > 0) {
        right_trigger = (float)rt / 32767.0f;
    } else {
        right_trigger = 0.0f;
    }
}
