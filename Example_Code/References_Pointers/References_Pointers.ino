

struct channel {
    float reading;
    float min;
    float max;
  };    

  class motor {
  public:
    motor(channel& ref) : ref_(ref) {
    // constructor code
    }

    void print() {
    //std::cout << "The value of the reference is: " << ref_.reading << std::endl;
    }

  private:
    channel& ref_;
  };

void setup() {
  float answer = 1;
  channel channel[6];   
  motor motor1(channel[0]);      
}

void loop() {
  //std::cout << "enter a value ";
  //std::cin >> answer;  
  channel[0].reading = answer;
  motor1.print();
    
}
