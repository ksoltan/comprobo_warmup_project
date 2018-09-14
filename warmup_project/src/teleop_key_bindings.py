"""
This file specifies the key bindings to the teleop program.
"""
key_actions = {
'a': 'MOVE_LEFT',
'd': 'MOVE_RIGHT',
'w': 'MOVE_FORWARD',
's': 'MOVE_BACKWARD',
'z': "TURN_LEFT",
'x': "TURN_RIGHT"
}

# Action bindings are in the format (linear.x, linear.y, linear.y, angular.z)
# Neato cannot do pitch and roll (angular.x/y)
action_bindings = {
'MOVE_LEFT' = (1, 0, 0, -1),
'MOVE_RIGHT' = (1, 0, 0, 1),
'MOVE_FORWARD' = (1, 0, 0, 0),
'MOVE_BACKWARD' = (-1, 0, 0, 1),
"TURN_LEFT" = (0, 0, 0, -1),
"TURN_RIGHT" = (0, 0, 0, 1)
}
