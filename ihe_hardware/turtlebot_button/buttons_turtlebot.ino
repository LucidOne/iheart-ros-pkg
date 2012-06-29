#include <ros.h>
#include <vector>
#include <turtlebot_button_op/Buttons.h>

ros::NodeHandle nh;

turtlebot_button_op::Buttons button_msg;
ros::Publisher pub_button("buttons", &button_msg);

const int num_buttons = 5;

int button0 = 8;
int button1 = 9;
int button2 = 10;
int button3 = 11;
int button4 = 12;

bool last_button_state[num_buttons];
int last_debounce_time[num_buttons];
int debounce_delay = 50;
bool published = true;
bool updated = false;

int i;

void setup() {
  nh.initNode();
  
  pinMode(button0, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  
  // Allocate space for button states
  button_msg.button_state_length = num_buttons;
  button_msg.button_state = (bool *) malloc(sizeof(bool) * num_buttons);
  
  // Get initial button state
  for (i = 0; i < num_buttons; i++) {
    // Buttons are normally low.
    last_button_state[i] = digitalRead(button0+i);
    last_debounce_time[i] = 0;
    
    button_msg.button_state[i] = last_button_state[i];
  }
  
  nh.advertise(pub_button);
}

void loop() {
  int reading[num_buttons];
  
  for (i = 0; i < num_buttons; i++) {
    reading[i] = digitalRead(button0+i);
  }
  
  
  for (i = 0; i < num_buttons; i++) {
    if (reading[i] != last_button_state[i]) {
      last_debounce_time[i] = millis();
      published = false;
      updated = true;
    }
  }
  
  for (i = 0; i < num_buttons; i++) {
    // Store stable button values
    if (!published && (millis() - last_debounce_time[i]) > debounce_delay) {
      button_msg.button_state[i] = reading[i];
    }
  }
  
  // Publish updated button message
  if (updated && !published) {
    pub_button.publish(&button_msg);
    published = true;
  }
  
  // Update last button state
  for (i = 0; i < num_buttons; i++) {
    last_button_state[i] = reading[i];
  }

  updated = false;
  
  nh.spinOnce();
}
