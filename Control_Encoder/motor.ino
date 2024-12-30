class motor {
  public:
    // Konstruktor
    motor(int pin_start, int pin_dir, float start_angle){
        input_pin_start = pin_start;
        input_pin_dir = pin_dir;
        current_angle = start_angle;
    }

    // Methoden
    void write_Info(){
        // Write info to motors
        digitalWrite(input_pin_start, start); 
        digitalWrite(input_pin_start, direction);
    }

    void check_if_target_reached(){
        if (direction){
            if (current_angle > target_angle){
                start = false; // stop the motor as soon as its reached the target angle
                digitalWrite(input_pin_start, start);
            }
            } else {
            if (current_angle < target_angle){
                start = false; // stop the motor as soon as its reached the target angle
                digitalWrite(input_pin_start, start);
            }
        }
    };

  private:
    // PINs
    int input_pin_start; // Output A
    int input_pin_dir; // Output B

    // Status Motor 
    bool start = false;
    bool direction = true;
    float current_angle = 90;
    float target_angle = 90;
};