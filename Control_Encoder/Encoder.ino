class encoder {
  public:
    // Konstruktor
    encoder(int pin_A, int pin_B){
        output_pin_A = pin_A;
        output_pin_B = pin_B;
    };

    // Methoden
    void read_pins(){
        pin_A_now = digitalRead(output_pin_A);
        pin_B_now = digitalRead(output_pin_B);
    }
    
    void update_encoder(){
        if ((pin_A_last == HIGH) && (pin_A_now == LOW)) {
            if (pin_B_now == HIGH) {
                angle++;
            } else {
                angle--;
            }
            }
        pin_A_last = pin_A_now;
    }


  private:
    // PINs
    int output_pin_A; // Output A
    int output_pin_B; // Output B

    // Status Encoder
    float angle = 0;
    int pin_A_last = 0;
    int pin_A_now = 0;
    int pin_B_now = 0;
};