# ROS Chess Engine


## Development Platform
I conducted this program's development on a machine running Ubuntu 20.04 and ROS Noetic  [Pyserial](https://pyserial.readthedocs.io/en/latest/pyserial.html) was used to create the serial driver.

# Architecture
The `ChessPlayer` class acts as a player in the chess game. It intiates the chess engine, keeps track of the chessboard state, takesturns with its opponent, and communicates with chess engine to make moeves. `ChessPlayer` is built on top of `SerialComms`, a simple serial driver class used to communicate with [Stockfish.](https://stockfishchess.org/). The original parameters for the chess challenge can be found [here](https://github.com/WHOIGit/ros-chess-challenge).

<p align="center">
  <img width="580" height="420" src="https://github.com/amfry/sp23-ros-chess-challenge/blob/main/docs/chess_opening_seq.png">
</p>
<p <p align="center">
  <em>The first 4 opening moves in a chess game between two instances of ChessPlayer.</em>
</p>

## Design Decisions
A major design decision I made was to store the state of the game in the nested dictionary. I selected the nested dictionary after interacting with the `chess/Chessboard` and `chess/Move` custom messages. To geneate the terminal rendering of the chessboard,'chess/Chessboard' contains 64 indivdual 'chess/Chesspiece'messages that are visualized `a1` through `h8` on the board. The outer dictionary of `board_state` contains an index that related to each spquare on the board starting `a1` through `h8`. At each of those index, I was able to store the relevant compnents to reprsent the state of the board. That included: 
  - row
  - colum
  - `chess/Chesspiece`
  
  When a new move was made, I was able to easily to find the index for the square impacted by the row and column conatined in `chess/Move` messages and could then update the piece at that position as needed. This also provided an easy way to check for capture and look at the conditions for castling.

## Castling
- Kingside
- Queenside
## Promotion
- Any UCI message of length 5 is a promotion

# Resources
- [Programming Robots with ROS](https://www.oreilly.com/library/view/programming-robots-with/9781449325480/)
- [A Gentle Introduction to ROS](https://jokane.net/agitr/)
- [How to Castle in Chess](https://www.chess.com/article/view/how-to-castle-in-chess)

# Future Improvments
If I had additional time and resources to dedicate to this project, I would extend this project in two major directions.

Error Checking and Testing: The testing present here does not necessarily encapsulate all use cases and potential opportunities for user error. For example, while there are some higher-level unit tests, I have no real end-to-end integration testing support in place that confirms that the server and client behave as expected. I also could add lower-level tests; for example, I could add unit tests for the compression algorithm itself to validate it against edge cases. I could also do more input validation.

Functionality: The current 
