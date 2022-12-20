# import gclib
import numpy as np
import time
import sys
import os
from functools import reduce
[sys.path.append(i) for i in['.','..']]
l=[]
script_path=os.path.split(sys.argv[0])
for i in range(len(script_path)):
    sys.path.append( reduce(os.path.join,script_path[:i+1]))

class NSCG3:
    """
    Python instrument driver for Newmark Systems NSCG3 motion controller. Uses
    Galil Motion Control API.

    Distance units are mm
    Speed units are mm/s
    Acceleration/deceleration units are mm/s^2

    The instrument API is accessible through self.inst

    The NSCG3 class contains an instance of axisController for each active axis.
    The motion control functions defined in NSCG3 will apply coordinated motion
    to all active axes. When sending motion commands with a list argument, the
    order of the elements in the list should correspond to the active axes in
    alphabetical order

    To control just a single axis when multiple axes are present, call the
    motion control functions from the axisController directly. For example.
    self.x.moveRelative(10) will move the X-axis 10 mm
    """
    def __init__(self,addr,activeAxes,simulate=False,debug=False):
        """
        Initialize the motion controller
        Inputs:
         addr: <str> IP address of instrument
         activeAxes: <str> string containing the active axes. ie. 'X', 'XZ',
                     'XYZ', etc... An AxisController object will be created for
                     each active axis and added to the list
                     self.axesControllers.
         simulate: <bool> run in simulation mode without hardware
         debug: <bool> print debug statements including commands sent to galil controller
        Returns:
         none
        """
        self.simulate = simulate
        self.debug = debug
        # Initialize the galil motion controller
        self.addr = addr
        # self.inst = gclib.py()
        self.inst = None
        if not self.simulate:
            self.inst.GOpen(addr+' --direct')

        # Initialize an axisController for each active axis
        self.activeAxes = activeAxes.upper()
        self.axesControllers = list()
        if(self.activeAxes.find('X') != -1):
            self.x = AxisController('X',self.inst,simulate=self.simulate,debug=self.debug)
            self.axesControllers.append(self.x)
        if(self.activeAxes.find('Y') != -1):
            self.y = AxisController('Y',self.inst,simulate=self.simulate,debug=self.debug)
            self.axesControllers.append(self.y)
        if(self.activeAxes.find('Z') != -1):
            self.z = AxisController('Z',self.inst,simulate=self.simulate,debug=self.debug)
            self.axesControllers.append(self.z)

        self.numAxes = len(self.axesControllers)

        self.speed = 20
        self.acceleration = 100
        self.deceleration = 100

    def sendCommand(self,command):
        """
        Send a command to the galil motion controller
        Inputs:
         command: <str> command to send
        Retrns:
         response: <str> response from motion controller
        """
        if self.debug:
            print(command)
        if not self.simulate:
            try:
                response = self.inst.GCommand(command)
                return response
            except:
                print('Error communicating with '+self.axis+' controller.  Attempting to stop all motion')
                self.inst.GCommand('ST')

    def getInfo(self):
        """
        Print info from motion controller
        Inputs:
         none
        Returns:
         none
        """
        if self.simulate:
            print('Simulated hardware')
        else:
            print(self.gc.GInfo())

    def setDebug(self,debug):
        self.debug = debug
        for axisController in self.axesControllers:
            axisController.debug = debug

    @property
    def position(self):
        """
        Get the position of all active axes. Position is read-only. To change the
        position, use the appropriate motion command such as jog, moveRelative,
        moveAbsolute, etc...
        Note: polling of position happens to each active axis sequentially. If
        the stage is moving when the command is issued, the reported positions
        may not be simultaneous. The function can be modified if simultaneous
        position values need to be returned
        Inputs:
         None
        Returns:
         position: <list> position of all active axes relative to zero (home
                   position unless reset by setZero()) in mm
        """
        position = list()
        for axisController in self.axesControllers:
            position.append(axisController.position)
        return position

    @property
    def forwardSoftwareLimit(self):
        """
        Get the software defined forward limit for all active axes
        Inputs:
         none
        Returns:
         limit: <float> limit relative to zero (home unless reset by setZero) in mm
        """
        limit = list()
        for axisController in self.axesControllers:
            limit.append(axisController.forwardSoftwareLimit)
        return limit

    @property
    def reverseSoftwareLimit(self):
        """
        Get the software defined reverse limit for all active axes
        Inputs:
         none
        Returns:
         limit: <float> limit relative to zero (home unless reset by setZero) in mm
        """
        limit = list()
        for axisController in self.axesControllers:
            limit.append(axisController.reverseSoftwareLimit)
        return limit

    @forwardSoftwareLimit.setter
    def forwardSoftwareLimit(self,val):
        """
        Sets the software defined forward limit for all axes.
        Inputs:
         val: <float> limit for all axes in mm relative to zero (home unless reset by setZero)
         val: <list> limit for each axis. Each entry sets the forward software
              limit for an individual axis. The order of the list corresponds to
              alphabetical order of the active axes
        Returns:
         none
        """
        if not isinstance(val,list):
            limit = [val for i in range(self.numAxes)]
        else:
            limit = val
        for i,axisController in enumerate(self.axesControllers):
            axisController.forwardSoftwareLimit = limit[i]

    @reverseSoftwareLimit.setter
    def reverseSoftwareLimit(self,val):
        """
        Sets the software defined reverse limit for all axes.
        Inputs:
         val: <float> limit for all axes in mm relative to zero (home unless reset by setZero)
         val: <list> limit for each axis. Each entry sets the reverse software
              limit for an individual axis. The order of the list corresponds to
              alphabetical order of the active axes
        Returns:
         none
        """
        if not isinstance(val,list):
            limit = [val for i in range(self.numAxes)]
        else:
            limit = val
        for i,axisController in enumerate(self.axesControllers):
            axisController.reverseSoftwareLimit = limit[i]

    def stop(self):
        """
        Stop motion on all active axes
        Inputs:
         none
        Returns:
         none
        """
        cmd = 'ST' + self.activeAxes
        self.sendCommand(cmd)

    def home(self):
        """
        Home all active axes. Home is defined relative to the reverse limit
        switch
        Inputs:
         none
        Returns:
         none
        """
        for axisController in self.axesControllers:
            # Set the speed and acceleration for each axis to the global speed
            axisController.speed = self.speed
            axisController.acceleration = self.acceleration
            axisController.deceleration = self.deceleration
            # Home all axes
            axisController.home()

    def wait(self):
        """
        Wait for motion to stop on all active axes
        Inputs:
         none
        Returns:
         none
        """
        if not self.simulate:
            self.inst.GMotionComplete(self.activeAxes)

    def _setRelativeKinematics(self,val):
        """
        Private function used to set the relative speeds and accelerations for
        each axis so that a move to a point follows a straight line to that
        point
        Inputs:
         val: <numpy array> vector describing the direction of motion
        Returns:
         none
        """

        # If the movement has zero length (ie. already at point) do nothing
        if np.linalg.norm(val) == 0:
            return

        # Compute relative speed, acceleration, and deceleration for each axis
        speeds = val/np.linalg.norm(val)*self.speed
        accelerations = val/np.linalg.norm(val)*self.acceleration
        decelerations = val/np.linalg.norm(val)*self.deceleration
        # Set the speed, acceleration, and deceleration for each axis
        for i,axisController in enumerate(self.axesControllers):
            # If the motion for this axis is zero counts, skip this step
            if int(val[i]*6250) == 0:
                continue
            axisController.speed = abs(speeds[i])
            axisController.acceleration = abs(accelerations[i])
            axisController.deceleration = abs(decelerations[i])

    def moveRelative(self,dist):
        """
        Move a distance relative to the current position on all active axes
        Inputs:
         dist: <list> list of relative distances. Each entry sets the relative
               distance for an individual axis. The order of the list corresponds
               to alphabetical order of the active axes
        Returns:
         none
        """
        if len(dist) != self.numAxes:
            print('List length does not match the number of active axes')
            return
        dist = np.array(dist)
        self._setRelativeKinematics(dist)
        for i,axisController in enumerate(self.axesControllers):
            axisController.moveRelative(dist[i],startImmediately=False)
        self.beginMotion()

    def moveAbsolute(self,pos):
        """
        Move to a position relative to zero (home position unless reset with
        setZero) on all active axes
        Inputs:
         pos: <list> list of absolute positions. Each entry sets the position
              for an individual axis. The order of the list corresponds to
              alphabetical order of the active axes
        Returns:
         none
        """
        if len(pos) != self.numAxes:
            print('List length does not match the number of active axes')
            return
        pos = np.array(pos)
        self._setRelativeKinematics(pos-np.array(self.position))
        for i,axisController in enumerate(self.axesControllers):
            axisController.moveAbsolute(pos[i],startImmediately=False)
        self.beginMotion()

    def jog(self,direction):
        """
        Jog on all active axes
        Inputs:
         direction: <list> direction vector
        Returns:
         none
        """
        if len(direction) != self.numAxes:
            print('List length does not match the number of active axes')
            return
        direction = np.array(direction)
        self._setRelativeKinematics(direction)
        for i,axisController in enumerate(self.axesControllers):
            if direction[i] < 0:
                axisController.jogBackward(startImmediately=False)
            else:
                axisController.jogForward(startImmediately=False)
        self.beginMotion()

    def beginMotion(self):
        """
        Starts motion on all active axes simultaneously
        Inputs:
         none
        Returnes:
         none
        """
        cmd = 'BG ' + self.activeAxes
        self.sendCommand(cmd)

    def setZero(self):
        """
        Sets the zero position for all active axes.
        Note: the command is sent sequentially to all axes, so if the stage is
        in motion, the zero point may not be defined exactly as desired. If
        needed, this function can be changed to zero all axes simultaneously
        Inputs:
         none
        Returns:
         none
        """
        for axisController in self.axesControllers:
            axisController.setZero()

    def resetForwardSoftwareLimit(self):
        """
        Resets the software defined forward limit to max value on all axes
        Inputs:
         none
        Returns:
         none
        """
        for axisController in self.axesControllers:
            axisController.resetForwardSoftwareLimit()

    def resetReverseSoftwareLimit(self):
        """
        Resets the software defined reverse limit to value min value on all axes
        Inputs:
         none
        Returns:
         none
        """
        for axisController in self.axesControllers:
            axisController.resetReverseSoftwareLimit()

class AxisController:
    """ 
    Python axis controller using galil motion control interface

    Distance units are mm
    Speed units are mm/s
    Acceleration/deceleration units are mm/s^2
    """
    def __init__(self,axis,inst,scaleFactor = 6250,simulate=False,debug=False):
        """
        Initialize axis controller
        Inputs:
         axis: <str> axis to initialize
         inst: <inst> reference to galil motion controller (from parent NSCG3)
         scaleFactor: <int> counts per mm
         simulate: <bool> simulate hardware
         debug: <bool> print debug statements including commands sent to galil controller
        """
        self.debug = debug
        self.simulate = simulate
        self.axis = axis

        # Dictionary to store command prefixes for each axis
        self.prefix = {
            'X': '',
            'Y': ',',
            'Z': ',,'
        }

        # Dictionary to convert axis names. Some Galil commands require the axes
        # to be named A,B,C instead of X,Y,Z
        self.axisConversion = {
            'X': 'A',
            'Y': 'B',
            'Z': 'C'
        }

        # Dictionary linking axis to thread number. Prevents two axes from
        # starting a program on the same thread
        self.thread = {
            'X': '0',
            'Y': '1',
            'Z': '2'
        }
        self.inst = inst
        self.scaleFactor = scaleFactor  # counts per mm
        self.simPosition = 0

    @property
    def forwardSoftwareLimit(self):
        """
        Get the software defined forward limit position
        Inputs:
         none
        Outputs:
         limit: <float> forward limit in mm
        """
        command = 'FL'+self.prefix[self.axis]+'?'
        limitStr = self.sendCommand(command)
        limit = float(limitStr)/self.scaleFactor
        return limit

    @property
    def reverseSoftwareLimit(self):
        """
        Get the software defined reverse limit position
        Inputs:
         none
        Outputs:
         limit: <float> reverse limit in mm
        """
        command = 'BL'+self.prefix[self.axis]+'?'
        limitStr = self.sendCommand(command)
        limit = float(limitStr)/self.scaleFactor
        return limit

    @property
    def forwardLimitState(self):
        """
        Get the forward limit switch state (read-only). The forward limit switch
        is the switch furthest from the motor
        Inputs:
         none
        Returns:
         state: <int> 0 - stage is beyond limit, 1 - stage is within limits
        """
        command = 'MG_LF'+self.axisConversion[self.axis]
        stateStr = self.sendCommand(command)
        state = int(round(float(stateStr)))
        return state

    @property
    def reverseLimitState(self):
        """
        Get the reverse limit switch state (read-only). The reverse limit switch
        is the switch closest to the motor
        Inputs:
         none
        Returns:
         state: <int> 0 - stage is beyond limit, 1 - stage is within limits
        """
        command = 'MG_LR'+self.axisConversion[self.axis]
        stateStr = self.sendCommand(command)
        state = int(round(float(stateStr)))
        return state

    @property
    def position(self):
        """
        Get the position of the axis controller (read-only)
        Inputs:
         none
        Returns:
         position: <float> position in mm relative to zero (home position unless
                   reset by setZero)
        """
        if self.simulate:
            return self.simPosition

        command = 'MG_RP'+self.axisConversion[self.axis]
        counts = self.sendCommand(command)
        position = float(counts)/self.scaleFactor
        return position

    @property
    def speed(self):
        """
        Get the speed of the axis controller
        Inputs:
         none
        Returns:
         speed: <float> speed in mm/s
        """
        command = 'SP'+self.prefix[self.axis]+'?'
        counts = self.sendCommand(command)
        speed = float(counts)/self.scaleFactor
        return speed

    @property
    def acceleration(self):
        """
        Get the acceleration of the axis controller
        Inputs:
         none
        Returns:
         acceleration: <float> acceleration in mm/s^2
        """
        command = 'AC'+self.prefix[self.axis]+'?'
        counts = self.sendCommand(command)
        acceleration = float(counts)/self.scaleFactor
        return acceleration

    @property
    def deceleration(self):
        """
        Get the deceleration of the axis controller
        Inputs:
         none
        Returns:
         deceleration: <float> deceleration in mm/s^2
        """
        command = 'DC'+self.prefix[self.axis]+'?'
        counts = self.sendCommand(command)
        deceleration = float(counts)/self.scaleFactor
        return deceleration

    @forwardSoftwareLimit.setter
    def forwardSoftwareLimit(self,pos):
        """
        Set the software defined forward limit
        Inputs:
         pos: <float> limit position in mm
        Returns:
         none
        """
        limitCounts = round(pos*self.scaleFactor)
        command = 'FL'+self.prefix[self.axis]+str(limitCounts)
        self.sendCommand(command)

    @reverseSoftwareLimit.setter
    def reverseSoftwareLimit(self,pos):
        """
        Set the software defined reverse limit
        Inputs:
         pos: <float> limit position in mm
        Returns:
         none
        """
        limitCounts = round(pos*self.scaleFactor)
        command = 'BL'+self.prefix[self.axis]+str(limitCounts)
        self.sendCommand(command)

    @speed.setter
    def speed(self,speed):
        """
        Set the speed of the axis controller
        Inputs:
         speed: <float> speed in mm/s
        Returns:
         none
        """
        speedCounts = int(speed*self.scaleFactor)
        command = 'SP'+self.prefix[self.axis]+str(speedCounts)
        self.sendCommand(command)

    @acceleration.setter
    def acceleration(self,acceleration):
        """
        Set the acceleration of the axis controller
        Inputs:
         acceleration: <float> acceleration in mm/s^2
        Returns:
         none
        """
        accelerationCounts = int(acceleration*self.scaleFactor)
        command = 'AC'+self.prefix[self.axis]+str(accelerationCounts)
        self.sendCommand(command)

    @deceleration.setter
    def deceleration(self,deceleration):
        """
        Set the deceleration of the axis controller
        Inputs:
         deceleration: <float> deceleration in mm/s^2
        Returns:
         none
        """
        decelerationCounts = int(deceleration*self.scaleFactor)
        command = 'DC'+self.prefix[self.axis]+str(decelerationCounts)
        self.sendCommand(command)

    def sendCommand(self,command):
        """
        Send a command to the galil motion controller
        Inputs:
         command: <str> command to send
        Retrns:
         response: <str> response from motion controller
        """
        if self.debug:
            print(command)
        if self.simulate:
            return 0
        else:
            try:
                response = self.inst.GCommand(command)
                return response
            except:
                print('Error communicating with '+self.axis+' controller.  Attempting to stop all motion')
                self.inst.GCommand('ST')

    def beginMotion(self):
        """
        Begin motion on axis controller
        Inputs:
         none
        Returns:
         none
        """
        command = 'BG'+self.axis
        self.sendCommand(command)

    def moveRelative(self,distance,startImmediately=True):
        """
        Move the axis controller a distance relative to the current position
        Inputs:
         distance: <float> distance in mm
         startImmediately: <bool> true - start motion immediately after issuing
                           command. false - issue command but don't start
                           motion. Used to initiate simultaneous motion after
                           commands are loaded on all axes
        Returns:
         none
        """
        if self.simulate:
            self.simPosition += distance
        counts = round(distance*self.scaleFactor)
        command = 'PR'+self.prefix[self.axis]+str(counts)
        self.sendCommand(command)
        if startImmediately:
            self.beginMotion()

    def moveAbsolute(self,pos,startImmediately=True):
        """
        Move to a position relative to zero (home position unless reset by self.setZero())
        Inputs:
         pos: <float> position in mm
         startImmediately: <bool> true - start motion immediately after issuing
                           command. false - issue command but don't start
                           motion. Used to initiate simultaneous motion after
                           commands are loaded on all axes
        Returns:
         none
        """
        if self.simulate:
            self.simPosition = pos
        counts = round(pos*self.scaleFactor)
        command = 'PA'+self.prefix[self.axis]+str(counts)
        self.sendCommand(command)
        if startImmediately:
            self.beginMotion()

    def jogForward(self,startImmediately=True):
        """
        Jog forward (away from the motor) at the current speed setting
        Inputs:
         startImmediately: <bool> true - start motion immediately after issuing
                           command. false - issue command but don't start
                           motion. Used to initiate simultaneous motion after
                           commands are loaded on all axes
        Returns:
         none
        """
        speedCounts = abs(round(self.speed*self.scaleFactor))
        command = 'JG'+self.prefix[self.axis]+str(speedCounts)
        self.sendCommand(command)
        if startImmediately:
            self.beginMotion()

    def jogBackward(self,startImmediately=True):
        """
        Jog backward (towards the motor) at the current speed setting
        Inputs:
         startImmediately: <bool> true - start motion immediately after issuing
                           command. false - issue command but don't start
                           motion. Used to initiate simultaneous motion after
                           commands are loaded on all axes
        Returns:
         none
        """
        speedCounts = -abs(round(self.speed*self.scaleFactor))
        command = 'JG'+self.prefix[self.axis]+str(speedCounts)
        self.sendCommand(command)
        if startImmediately:
            self.beginMotion()

    def home(self):
        """
        Home the axis controller. Home is defined relatie to the reverse limit
        switch. The home program is downloaded onto the motion controller. To
        change it, modify home.g and download it to the controller using
        self.inst.GProgramDownloadFile('home.g'). After testing, burn it to the
        controllers non-volataile memory using self.inst.GCommand('BP')
        Inputs:
         none
        Outputs:
         none
        """
        # Execute the home program on the controller
        command = 'XQ#HOME'+self.axis+','+self.thread[self.axis]
        self.sendCommand(command)

    def wait(self):
        """
        Wait until motion on this axis completes
        Inputs:
         none
        Returns:
         none
        """
        if not self.simulate:
            self.inst.GMotionComplete(self.axis)

    def stop(self):
        """
        Stop motion on this axis
        Inputs:
         none
        Returns:
         none
        """
        command = 'ST'+self.axis
        self.sendCommand(command)

    def setZero(self):
        """
        Zero the axis controller to the current position
        Inputs:
         none
        Returns:
         none
        """
        command = 'DP'+self.prefix[self.axis]+str(0)
        self.sendCommand(command)

    def resetForwardSoftwareLimit(self):
        """
        Reset the forward software defined limit to max value
        Inputs:
         none
        Returns:
         none
        """
        maxValue = 2147483647
        command = 'FL'+self.prefix[self.axis]+str(maxValue)
        self.sendCommand(command)

    def resetReverseSoftwareLimit(self):
        """
        Reset the reverse software defined limit to min value
        Inputs:
         none
        Returns:
         none
        """
        minValue = -2147483648
        command = 'BL'+self.prefix[self.axis]+str(minValue)
        self.sendCommand(command)

# n = NSCG3('169.254.154.15','XYZ',simulate=False)

