#pragma once
#include <SDL3/SDL.h>

class GamepadInput {
public:
    GamepadInput();
    ~GamepadInput();

    void update();

    // -1.0 to 1.0 (deadzoned)
    float getLeftX() const { return left_x; }
    float getLeftY() const { return left_y; }

    // 0.0 to 1.0
    float getRightTrigger() const { return right_trigger; }

    bool isConnected() const { return gamepad != nullptr; }

private:
    SDL_Gamepad* gamepad = nullptr;
    float left_x = 0.0f;
    float left_y = 0.0f;
    float right_trigger = 0.0f;

    void openGamepad();
    float applyDeadzone(int16_t axis_val, int16_t deadzone = 8000);
};
