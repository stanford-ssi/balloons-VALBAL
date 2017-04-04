#!/usr/bin/env bash
# Stanford Student Space Initiative
# Balloons | VALBAL | April 2017
# Davy Ragland | dragland@stanford.edu
# Aria Tedjarati | satedjarati@stanford.edu
# Joan Creus-Costa | jcreus@stanford.edu
# Claire Huang | chuang20@stanford.edu
# Michal Adamkiewicz | mikadam@stanford.edu
# Jesus Cervantes | cerjesus@stanford.edu
# Matthew Tan | mratan@stanford.edu

# File: simulator.sh
# --------------------------
# Server side script to feed simuled data to VALBAL
# over Serial for Hardware in the Loop testing.

echo -e "simulated\nlines\nfor\ntesting" > /dev/ttyACM0

# void getLine() {
#   while(true) {
#     if(Serial.available()) {
#       char c = Serial.read();
#       if(c == '\n') {
#         Serial.print('\n');
#         return;
#       }
#       Serial.print(c);
#     }
#   }
# }
#
# int main() {
#   Serial.begin(115200);
#   while(true) {
#     getLine();
#   }
# }
