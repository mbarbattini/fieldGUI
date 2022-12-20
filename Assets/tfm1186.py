import pyvisa as visa
import numpy as np

class TFM1186:
    """
    Python instrument driver for MetroLab TFM1186 Fluxgate magnetometer.
    Using VISA driver with SCPI commands

    The instrument API is accessible through self.inst
    """

    def __init__(self,addr,averages=1,simulate=False):
        """
        Initialize instrument.
        Inputs:
         addr: <string> address of instrument
         averages: <int> number of onboard averages
         simulate: <bool> simulate hardware flag
        Returns:
         none
        """
        self.addr = addr
        self.inst = None
        self.simulate = simulate

        # Create dictionary for simulated data (noise will be added in the measurement section)
        self.simulatedData = {'X':12349,
                              'Y':3982,
                              'Z':42761}

        if not self.simulate:
            self.connect() # run 'connect' method on instantiation
            self.inst.timeout = 10000
            self.setFormat('INT')
            self.setAverages(averages)

    def __del__(self):
        """
        Destructor.  Closes inst connection
        """
        if not self.simulate:
            self.inst.close()

    def connect(self):
        """
        Connect to instrument on address self.addr
        """
        rm = visa.ResourceManager()
        rlist = rm.list_resources()
        if self.addr in rlist:
            self.inst = rm.open_resource(self.addr)


    def setFormat(self,val):
        return self.write('FORM:DATA ' + val)

    def getAverages(self):
        return self.query('CALC:AVER:COUNT?')

    def setAverages(self,val):
        return self.write('CALC:AVER:COUN '+str(val))

    def write(self,val):
        """
        Write a command to the instrument
        Inputs:
         val: <string> command to be written
        """
        return self.inst.write(val)

    def query(self,val):
        return self.inst.query(val)

    def queryBinary(self,val):
        return self.inst.query_binary_values(val)

    def getIDN(self):
        return self.query('*IDN?')

    def measureOnce(self,val,digits=5):
        """
        Get a single measurement from the magnetometer
        Inputs
         val: <char> direction - 'X', 'Y', or 'Z'
         digits: <int> number of digits of precision - min:1, max:5
        Outputs
         meas: <int> field value in nT
         saturated: <bool> flag indicating if measurement range exceeded
        """
        saturated = False
        if self.simulate:
            sig = self.simulatedData[val]
            noise = np.random.randn(1)[0]*10
            meas = np.round(sig+noise)
        else:
            if digits > 5:
                print("digits must be an integer between 1 and 5")
                return
            meas = self.inst.query_binary_values('READ:SCALAR:FLUX:'+val+'? ,'+str(digits),datatype=u'i',is_big_endian=True)
            if np.abs(meas) > 100000:
                saturated = True
        return(meas,saturated)

    def measureBlock(self,blockSize,val,digits=5):
        """
        Get a block of measurements from the magnetometer
        Inputs
         blockSize: <int> number of samples in block
         val: <char> direction - 'X', 'Y', or 'Z'
         digits: <int> number of digits of precision - min:1, max:5
        Outputs
         meas: <int> field value in nT
         saturated: <bool> flag indicating if any measurement
             exceeeded measurement range
        """

        saturated = False
        if self.simulate:
            sig = self.simulatedData[val]*np.ones(blockSize)
            noise = np.random.randn(blockSize)*10
            meas = sig+noise
        else:
            if digits > 5:
                print("digits must be an integer between 1 and 5")
                return
            meas = self.inst.query_binary_values('READ:ARRAY:FLUX:'+val+'? '+str(blockSize)+',,'+str(digits),datatype=u'i',is_big_endian=True)
            if(np.any(np.abs(np.array(meas)) > 100000)):
                saturated = True
        return(meas,saturated)

    # Revist this later - not sure why it's needed
    def fetchBlock(self,blockSize,val,digits=5):
        if digits > 5:
            print("digits must be an integer between 1 and 5")
            return
        return self.inst.query_binary_values('FETCH:ARRAY:FLUX:'+val+'? '+str(blockSize)+','+str(digits),datatype=u'i',is_big_endian=True)


    @staticmethod
    def GetResources():
        """
        Get a list of visa resources
        Outputs:
         resources: <list> list of visa resources
        """
        rm = visa.ResourceManager()
        return rm.list_resources()
