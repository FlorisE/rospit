""" Numeric evaluation, e.g. a function with limits """

from abc import ABCMeta, abstractmethod
from collections import namedtuple
import future  # noqa:401, pylint: disable=W0611

from rospit.framework import Evaluator, Evaluation, CompositeEvaluation, Sensor, \
                             Measurement, Condition


def numeric_measure(evaluator):
    measurement = evaluator.call_evaluator()
    if isinstance(measurement, int) or isinstance(measurement, float):
        return NumericMeasurement(measurement)
    elif isinstance(measurement, NumericMeasurement):
        return measurement
    else:
        raise TypeError("measurement should either be a float, an int or a NumericMeasurement")

class LowerLimitEvaluator(Evaluator):
    """
    Evaluator for the lower limit of numeric conditions
    """
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit condition
        """
        if measurement is None:
            measurement = numeric_measure(self)

        if condition.lower_limit_is_inclusive:
            nominal = measurement.value >= condition.lower_limit
        else:
            nominal = measurement.value > condition.lower_limit

        return Evaluation(measurement, condition, nominal)


class UpperLimitEvaluator(Evaluator):
    """
    Evaluator for the upper limit of numeric conditions
    """
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the upper limit condition
        """
        if measurement is None:
            measurement = numeric_measure(self)

        if condition.upper_limit_is_inclusive:
            nominal = measurement.value <= condition.upper_limit
        else:
            nominal = measurement.value < condition.upper_limit

        return Evaluation(measurement, condition, nominal)


class BothLimitsEvaluator(LowerLimitEvaluator, UpperLimitEvaluator):
    """
    Evaluator for numeric conditions
    """
    def __init__(self, evaluator):
        LowerLimitEvaluator.__init__(self, evaluator)
        UpperLimitEvaluator.__init__(self, evaluator)

    def evaluate(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit and
        upper limit conditions
        """
        if measurement is None:
            measurement = numeric_measure(self)

        lower_limit_eval = LowerLimitEvaluator.evaluate(
            self, condition, measurement)
        upper_limit_eval = UpperLimitEvaluator.evaluate(
            self, condition, measurement)

        return CompositeEvaluation(
            measurement, condition,
            [lower_limit_eval, upper_limit_eval])


class NumericSensor(Sensor):
    """
    Sensor that reads a numeric value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense(self):
        pass


class NumericMeasurement(Measurement):
    """
    Measurement of a numeric value
    """
    def __init__(self, value):
        self.value = value


Limit = namedtuple("Limit", "limit is_inclusive")


def get_inclusive_limit(value):
    """
    Gets an inclusive limit at the specified value
    >>> limit = get_inclusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    True
    """
    return Limit(value, True)


def get_exclusive_limit(value):
    """
    Gets an exclusive limit at the specified value
    >>> limit = get_exlusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    False
    """
    return Limit(value, False)


def try_get_limit(limit):
    limit_type = type(limit)
    if limit_type is float or limit_type is int:
        return (limit, True)
    elif limit_type is Limit:
        return (lower_limit.limit, lower_limit.is_inclusive)
    else:
        raise TypeError("limit needs to be either an instance of Limit, an int or a float")

class LowerLimitCondition(Condition):
    """
    A condition for a numeric function with just a lower limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, name=""):
        Condition.__init__(self, name)
        self.lower_limit, self.lower_limit_is_inclusive = try_get_limit(lower_limit)


class UpperLimitCondition(Condition):
    """
    A condition for a numeric function with just an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, upper_limit, name=""):
        Condition.__init__(self, name)
        self.upper_limit, self.upper_limit_is_inclusive = try_get_limit(upper_limit)


class BothLimitsCondition(LowerLimitCondition, UpperLimitCondition):
    """
    A condition for a numeric function with a lower limit and an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, upper_limit, name=""):
        LowerLimitCondition.__init__(self, lower_limit, name)
        UpperLimitCondition.__init__(self, upper_limit, name)
