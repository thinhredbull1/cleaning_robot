Header header

# True while a goal is being executed
bool active

# Mode of the running goal, see MoveCommand.action
uint8 mode
string mode_name

# Internal state of the running goal, see MoveCommand.action
uint8 state
string state_name

string message
