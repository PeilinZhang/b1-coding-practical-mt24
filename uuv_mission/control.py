# control.py

class PDController:
    def __init__(self, kp: float, kd: float):
        """
        Proportional-Derivative (PD) controller
        :param kp: Proportional gain
        :param kd: Derivative gain
        """
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0  # To store the error from the previous time step

    def control(self, reference: float, observation: float, dt: float) -> float:
        """
        Compute the control action using the PD formula.
        :param reference: Desired target value (set point)
        :param observation: Current observed value
        :param dt: Time step (time delta)
        :return: Control action to apply to the system
        """
        # Calculate the error
        error = reference - observation

        # Calculate the derivative of the error
        derivative = (error - self.previous_error) / dt

        # PD control law
        action = self.kp * error + self.kd * derivative

        # Update previous error
        self.previous_error = error

        return action
