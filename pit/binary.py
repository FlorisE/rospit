""" Binary conditions """

from abc import ABCMeta, abstractmethod

from pit.framework import Evaluator, Evaluation, Measurement, Condition, Sensor


class BinaryConditionEvaluator(Evaluator):
    """
    Evaluator for binary values
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = self.sensor.sense()
        nominal = measurement.value == condition.value
        return Evaluation(measurement, condition, nominal)


class BinarySensor(Sensor):
    """
    Sensor that returns a binary value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense(self):
        pass


class BinaryMeasurement(Measurement):
    """
    Measurement of a binary value
    """
    def __init__(self, value):
        self.value = value


class BinaryCondition(Condition):
    """
    Condition that is either True or False
    """
    __metaclass__ = ABCMeta

    def __init__(self, value, name=""):
        Condition.__init__(self, name)
        self.value = value
