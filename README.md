# ROS Chess Engine
## Development Platform
I conducted this program's development on a machine running Ubuntu 20.04, Python3 and ROS Noetic.  [Pyserial](https://pyserial.readthedocs.io/en/latest/pyserial.html) was used to create the serial driver.

# Architecture
The `ChessPlayer` class acts as a player in the chess game. It initiates the chess engine, keeps track of the chessboard state, takes turns with its opponent, and communicates with the chess engine to make moeves. `ChessPlayer` is built on top of `SerialComms`, a simple serial driver class used to communicate with [Stockfish.](https://stockfishchess.org/) The original parameters for the chess challenge can be found [here](https://github.com/WHOIGit/ros-chess-challenge).

<p align="center">
  <img width="580" height="420" src="https://github.com/amfry/sp23-ros-chess-challenge/blob/main/docs/chess_opening_seq.png">
</p>
<p <p align="center">
  <em>The first 4 opening moves in a chess game between two instances of ChessPlayer.</em>
</p>

## Design Decisions
A major design decision I made was how to store the state of the game.  I chose a nested dictionary because it allowed me to store many dictionaries, each representing a square of the board at an key that also related to the layout of the board. To generate the terminal rendering of the chessboard,'chess/Chessboard' contains 64 individual `chess/Chesspiece` messages that are visualized `a1` through `h8` on the board. The outer dictionary of `board_state` contains an index that relates to each square on the board starting `a1` through `h8`. At each of those indexes, I was able to store the relevant components to represent the state of the board. That included: 
  - row
  - colum
  - `chess/Chesspiece`
  
  When a new move was made, I was able to easily find the index for the square impacted by the row and column contained in `chess/Move` messages and could then update the piece at that position as needed. This also provided an easy way to check for capture and look at the conditions for castling.

# Resources
I referenced the following materials to refresh on ROS debugging tool and learn more about using custom messages.
- [Programming Robots with ROS](https://www.oreilly.com/library/view/programming-robots-with/9781449325480/)
- [A Gentle Introduction to ROS](https://jokane.net/agitr/) 

I referenced this resource to get more familiar with the rules of chess and castling.
- [How to Castle in Chess](https://www.chess.com/article/view/how-to-castle-in-chess)

# Future Improvements
If I had additional time and resources to dedicate to this project, I would extend this project in two major directions.

Error Checking and Testing: The testing present here does not encapsulate all use cases and potential chess game scenarios. The chessboard visual in the terminal was a helpful tool for debugging and confirming nominal operation. However, there is limited error handling for things like unexpected characters in UCI strings which makes the overall design more brittle than I would like. Additionally, the same or very similar opening sequences were continually played by the chess engine. With more time, I would like to dig into using ROStest to inject more diversity into the game scenarios to excercise a fuller range of outcomes.

Functionality: The current implementation does not handle check, checkmate,and more complex castling scenarios. Right now, the player node assumes that if the king is moving to a kingside or queenside position, then the move is castling and the rooks position is also updated. This overlooks scenarios where castling would not be permitted because of the king or rook having been moved, pieces being between the king and rook, or the king being in check/checkmate. Being able to handle check and checkmate would create a clear winner and exit condition for the game as well as being more inline with castling rules.

