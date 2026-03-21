// Regular class ------------------

class Morse {
public:
  Morse(int pin) {
    pinMode(pin, OUTPUT);
    _pin = pin;
  }
  void dot() {
    digitalWrite(_pin, HIGH);
    delay(250);
    digitalWrite(_pin, LOW);
    delay(250);
  }
  void dash() {
    digitalWrite(_pin, HIGH);
    delay(1000);
    digitalWrite(_pin, LOW);
    delay(250);
  }

private:
  int _pin;
};

Morse morse(13);

// Inheritence & Polymorphism -------------------

class Animal { // Parent or abstract class
  public:
    virtual void talk() = 0; // pure virtual function
    virtual void walk() { //normal method
      Serial.println("Animal Walking");
    }
};

class Dog : public Animal {
  public:
    void talk() {
      Serial.println("Woof!");
    }
};

class Duck : public Animal {
  public:
    void talk() {
      Serial.println("Quack!");
    }
    void fly() {
      Serial.println("Duck flying!");
    }
};

Animal *baxter = new Dog();
Animal *donald = new Duck();
Animal *group[] = {baxter, donald};

// -------------------------

void setup() {
  
  Serial.begin(9600);
  
  for(auto &item : group){
    item->talk();
  }


}




void loop() {
  
}



