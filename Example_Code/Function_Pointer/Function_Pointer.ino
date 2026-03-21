void a() {
  Serial.println("a");
}

void b() {
  Serial.println("b");
}

void c() {
  Serial.println("c");
}

void A() {
  Serial.println("A");
}

void B() {
  Serial.println("B");
}

void C() {
  Serial.println("C");
}

void task1() {
  Serial.println("Task1");
}
void task2() {
  Serial.println("Task2");
}
void task3() {
  Serial.println("Task3");
}

//Function Pointer Arrays

typedef void (*functionPointer)();

functionPointer callTask[] = {
  task1,
  task2,
  task3
};

functionPointer lowerCase[] = {
  a,
  b,
  c
};

functionPointer upperCase[] = {
  A,
  B,
  C
};

void setup() {
  Serial.begin(9600);
  for (byte j = 0; j < 3; j++) {
    callTask[j]();  //need index and extra brackets()to call the function
  }
}

void loop() {
  char ch;
  if (Serial.available() > 0) {
    ch = Serial.read();
    if (ch >= 'a' && ch <= 'z') {
      (lowerCase[ch - 'a'])();
    } else if (ch >= 'A' && ch <= 'Z') {
      (upperCase[ch - 'A'])();
    }
    //else if (ch >= '0' && ch <= '9')
    //{
    //  (*Numbers[ch - '0'])();
    //}
    else if (ch == ' ') {
    }
  }
}