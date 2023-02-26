#ROS Chess Engine

Original Challenge: [here](https://github.com/WHOIGit/ros-chess-challenge)
#Development Platform
I conducted this program's development on a machine running Ubuntu 20.04 and ROS Noetic.

#Architecture
- General purpose serial library
- Player node

## Design Decisions
- Game state in nested dictionary

## Castling
- Kingside
- Queenside
## Promotion
- Any UCI message of length 5 is a promotion

#Resources
# Libraries
#Future Improvments
- Functionality: Handle check & checkmate so that the game can come to an end!
