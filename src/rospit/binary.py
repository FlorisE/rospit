""" Binary conditions """

from abc import ABCMeta, abstractmethod

from rospit.framework import Evaluator, Evaluation, Measurement, Condition, Sensor


class BinaryConditionEvaluator(Evaluator):
    """
    Evaluator for binary values
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)
        self.last_measurement_value = None
        self.last_condition_value = None

    def evaluate(self, condition, measurement=None):
        if measurement is None:
            measurement = self.call_evaluator()

        if isinstance(measurement, BinaryMeasurement):
            self.last_measurement_value = measurement.value
        elif isinstance(measurement, bool):
            self.last_measurement_value = measurement
        else:
            raise Exception("Measurement is of unexpected type")

        if isinstance(condition, BinaryCondition):
            self.last_condition_value = condition.value
        elif isinstance(condition, bool):
           self.last_condition_value = condition
        else:
            raise Exception("Condition is of unexpected type")

        nominal = self.last_measurement_value == self.last_condition_value
        return Evaluation(measurement, condition, nominal)


class BinarySensor(Sensor):
    """
    Sensor that returns a binary value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
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
