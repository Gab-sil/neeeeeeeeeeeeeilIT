#include <Servo.h>
#include "hardware/pwm.h"

#define IR_PIN 14
#define SERVO_PIN 0

Servo myServo;

void ir_pwm_init() {
    gpio_set_function(IR_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(IR_PIN);
    
    // 125MHz / 3289 ≈ 38kHz
    pwm_set_wrap(slice, 3289);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_gpio_level(IR_PIN, 3289 / 2); // 50% duty cycle
    pwm_set_enabled(slice, true);
}

void setup() {
    Serial.begin(115200);
    
    ir_pwm_init();
    
    myServo.attach(SERVO_PIN);
    myServo.write(90); // posição central
    
          // put your main code here, to run repeatedly:
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);

    Serial.println("IR slice: " + String(pwm_gpio_to_slice_num(IR_PIN)));
    Serial.println("Servo slice: " + String(pwm_gpio_to_slice_num(SERVO_PIN)));
}

void loop() {
    // varre o servo de 0 a 180 enquanto o IR emite continuamente
    for (int pos = 0; pos <= 180; pos++) {
        myServo.write(pos);
        delay(15);
    }
    for (int pos = 180; pos >= 0; pos--) {
        myServo.write(pos);
        delay(15);
    }
}
