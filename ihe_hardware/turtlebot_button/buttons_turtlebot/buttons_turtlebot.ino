#include <ros.h>
#include <turtlebot_button/Buttons.h>

ros::NodeHandle nh;

turtlebot_button::Buttons button_msg;
ros::Publisher pub_button("buttons", &button_msg);

const int num_buttons = 5;

const int estop = 7;
const int button0 = 8;
const int button1 = 9;
const int button2 = 10;
const int button3 = 11;
const int button4 = 12;

int last_debounce_time[num_buttons];
int debounce_delay = 50;
bool button_updated[num_buttons];
bool published = true;

int i;

void setup() {
  nh.initNode();
  nh.advertise(pub_button);
  
  pinMode(estop, INPUT);
  pinMode(button0, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  
  // Allocate space for button states
  button_msg.button_state_length = num_buttons;
  button_msg.button_state = (uint8_t *) malloc(sizeof(uint8_t) * num_buttons);
  
  // Get initial state
  // Depressed estop returns 0; otherwise, 1
  button_msg.estop_state = digitalRead(estop); 
  
  for (i = 0; i < num_buttons; i++) {
    // Buttons are normally high.
    button_msg.button_state[i] = !digitalRead(button0+i);
    last_debounce_time[i] = 0;
    button_updated[i] = false;
  }
  
  // 35-second delay before reliable data is published
  delay(35000);
}

void loop() {
  int button_reading[num_buttons];
  int estop_reading;
  
  // Get current state
  estop_reading = digitalRead(estop);
  
  for (i = 0; i < num_buttons; i++) {
    button_reading[i] = !digitalRead(button0+i);
  }
  
  // Reset debounce timer if button state changed
  for (i = 0; i < num_buttons; i++) {
    if (button_reading[i] != button_msg.button_state[i]) {
      last_debounce_time[i] = millis();
      button_updated[i] = true;
    }
  }
  
  // Store stable button values
  // Publish new message only when values have been updated
  for (i = 0; i < num_buttons; i++) {
    if (button_updated[i] && (millis() - last_debounce_time[i]) > debounce_delay) {
      button_msg.button_state[i] = button_reading[i];
      button_updated[i] = false;
      published = false;
    }
  }
  
  if (estop_reading != button_msg.estop_state) {
    button_msg.estop_state = estop_reading;
    published = false;
  }
  
  // Publish updated button message
  if (!published) {
    pub_button.publish(&button_msg);
    published = true;
  }
  
  nh.spinOnce();
}
