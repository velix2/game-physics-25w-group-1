#include "Scene.h"
#include "Common.h"

class SceneInteractive : public Scene {
    TempField temp_field = TempField(0.1f, generateSineWaveField(16, 16, 350.0f, 2.5f), 0, 1, 0, 1);
    float delta_t = 0.005;
    float rendering_horizontal_scale = 10.0f;
    float rendering_vertical_scale = 0.01f;
    float min_temp = -500.0f, max_temp = 500.0f;
    bool is_running = false;
    bool is_using_implicit = true;

    // Reinit values
    int selected_method = 1;

    // Random
    float rand_min_val = -250.0f, rand_max_val = 250.0f;

    // Sine Wave
    float sine_amplitude = 350.0f, sine_repetitions = 2.5f;

    // Gaussian
    float gaussian_mean_pos[2] = {0.5f, 0.5f};
    float gaussian_amplitude = 250, gaussian_sigma = 0.2f;

    // Common
    int m = 16, n = 16;
    float x_domain_lower_bound = 0, x_domain_upper_bound = 1; 
    float y_domain_lower_bound = 0, y_domain_upper_bound = 1; 

    void simulateStep() override;
    void onGUI() override;
    void onDraw(Renderer &r) override;
};