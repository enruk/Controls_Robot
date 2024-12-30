#ifndef MYLIBRARY_H
#define MYLIBRARY_H

// Deklaration der Klasse
class motor {
  public:
    // Konstruktor
    motor(int pin_start, int pin_dir, float current_angle);

    // Methoden
    void write_Info();
    void check_if_target_reached();

  private:
    // PINs
    int input_pin_start; // Output A
    int input_pin_dir; // Output B

    // Status Motor 
    bool start = false;
    bool dir = true;
    float current_angle = 90;
    float target_angle = 90;
    
};

#endif
