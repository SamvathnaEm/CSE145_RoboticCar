from mpi_control import MegaPiController

# Create an instance of the MegaPiController
mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)

# Test if the controller works by making the robot go straight
mpi_ctrl.carStraight(50)

