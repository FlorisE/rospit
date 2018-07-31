""" Tests for numeric evaluators """

import unittest
import add_sut
from rospit.numeric import NumericSensor, \
                           NumericMeasurement, \
                           LowerLimitCondition, \
                           LowerLimitEvaluator, \
                           UpperLimitCondition, \
                           UpperLimitEvaluator, \
                           BothLimitsCondition, \
                           BothLimitsEvaluator, \
                           get_inclusive_limit, \
                           get_exclusive_limit


BELOW_LOWER_LIMIT_VALUE = -3
LOWER_LIMIT_VALUE = -2
BETWEEN_LOWER_LIMIT_AND_EXPECTED_VALUE = -1
EXPECTED_VALUE = 0
BETWEEN_EXPECTED_AND_UPPER_LIMIT_VALUE = 1
UPPER_LIMIT_VALUE = 2
ABOVE_UPPER_LIMIT_VALUE = 3


class BelowLowerLimitNumericSensor(NumericSensor):
    """
    Senses a value that is below the lower limit
    """
    def sense_internal(self):
        return NumericMeasurement(BELOW_LOWER_LIMIT_VALUE)


class OnLowerLimitNumericSensor(NumericSensor):
    """
    Senses a value that is on the lower limit
    """
    def sense_internal(self):
        return NumericMeasurement(LOWER_LIMIT_VALUE)


class BetweenLowerLimitAndExpectedValueNumericSensor(NumericSensor):
    """
    Senses a value that is above the lower limit but below the expected value
    """
    def sense_internal(self):
        return NumericMeasurement(BETWEEN_LOWER_LIMIT_AND_EXPECTED_VALUE)


class ExpectedValueNumericSensor(NumericSensor):
    """
    Senses the expected value
    """
    def sense_internal(self):
        return NumericMeasurement(EXPECTED_VALUE)


class BetweenExpectedValueAndUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is above the expected value but below the upper limit
    """
    def sense_internal(self):
        return NumericMeasurement(BETWEEN_EXPECTED_AND_UPPER_LIMIT_VALUE)


class OnUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is on the upper limit
    """
    def sense_internal(self):
        return NumericMeasurement(UPPER_LIMIT_VALUE)


class AboveUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is above the upper limit
    """
    def sense_internal(self):
        return NumericMeasurement(ABOVE_UPPER_LIMIT_VALUE)


def get_numeric_sensor(value):
    """
    Gets a sensor which measures the desired value
    """
    if value < LOWER_LIMIT_VALUE:
        sensor = BelowLowerLimitNumericSensor()
    elif value > UPPER_LIMIT_VALUE:
        sensor = AboveUpperLimitNumericSensor()
    elif value == LOWER_LIMIT_VALUE:
        sensor = OnLowerLimitNumericSensor()
    elif value == UPPER_LIMIT_VALUE:
        sensor = OnUpperLimitNumericSensor()
    elif value < EXPECTED_VALUE:
        sensor = BetweenLowerLimitAndExpectedValueNumericSensor()
    elif value > EXPECTED_VALUE:
        sensor = BetweenExpectedValueAndUpperLimitNumericSensor()
    else:
        sensor = ExpectedValueNumericSensor()
    return sensor


def evaluate_lower_limit_condition(value, lower_limit):
    """
    Evaluates a lower limit condition
    """
    sensor = get_numeric_sensor(value)
    condition = LowerLimitCondition(lower_limit)
    evaluator = LowerLimitEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


def evaluate_upper_limit_condition(value, upper_limit):
    """
    Evaluates an upper limit condition
    """
    sensor = get_numeric_sensor(value)
    condition = UpperLimitCondition(upper_limit)
    evaluator = UpperLimitEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


def evaluate_both_limits_condition(value, lower_limit, upper_limit):
    """
    Evaluate a condition with a lower and an upper limit
    """
    sensor = get_numeric_sensor(value)
    condition = BothLimitsCondition(lower_limit, upper_limit)
    evaluator = BothLimitsEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


INCLUSIVE_LOWER_LIMIT = get_inclusive_limit(LOWER_LIMIT_VALUE)
INCLUSIVE_UPPER_LIMIT = get_inclusive_limit(UPPER_LIMIT_VALUE)
EXCLUSIVE_LOWER_LIMIT = get_exclusive_limit(LOWER_LIMIT_VALUE)
EXCLUSIVE_UPPER_LIMIT = get_exclusive_limit(UPPER_LIMIT_VALUE)


class LowerLimitEvaluatorTest(unittest.TestCase):
    """
    Tests for the lower limit evaluator
    """
    def test_nominal_if_on_lower_and_inclusive(self):
        """
        Nominal if the measurement is on the lower limit and
        the limit is inclusive
        """
        _, _, _, evaluation = evaluate_lower_limit_condition(
            LOWER_LIMIT_VALUE, INCLUSIVE_LOWER_LIMIT)
        self.assertTrue(evaluation.nominal)

    def test_not_nominal_if_on_lower_and_not_inclusive(self):
        """
        Not nominal if the measurement is on the lower limit and
        the limit is not inclusive
        """
        _, _, _, evaluation = evaluate_lower_limit_condition(
            LOWER_LIMIT_VALUE, EXCLUSIVE_LOWER_LIMIT)
        self.assertFalse(evaluation.nominal)

    def test_nominal_if_between_lower_and_expected(self):
        """
        Nominal if the measurement is between the lower limit
        and the expected value
        """
        _, _, _, evaluation = evaluate_lower_limit_condition(
            BETWEEN_LOWER_LIMIT_AND_EXPECTED_VALUE, EXCLUSIVE_LOWER_LIMIT)
        self.assertTrue(evaluation.nominal)


class UpperLimitEvaluatorTest(unittest.TestCase):
    """
    Tests for the upper limit evaluator
    """
    def test_nominal_if_on_upper_and_inclusive(self):
        """
        Nominal if the measurement is on the upper limit and
        the limit is inclusive
        """
        _, _, _, evaluation = evaluate_upper_limit_condition(
            UPPER_LIMIT_VALUE, INCLUSIVE_UPPER_LIMIT)
        self.assertTrue(evaluation.nominal)

    def test_not_nominal_if_on_upper_and_not_inclusive(self):
        """
        Not nominal if the measurement is on the lower limit and
        the limit is not inclusive
        """
        _, _, _, evaluation = evaluate_upper_limit_condition(
            UPPER_LIMIT_VALUE, EXCLUSIVE_UPPER_LIMIT)
        self.assertFalse(evaluation.nominal)

    def test_nominal_if_between_expected_and_upper(self):
        """
        Nominal if the measurement is between the lower limit
        and the expected value
        """
        _, _, _, evaluation = evaluate_upper_limit_condition(
            BETWEEN_EXPECTED_AND_UPPER_LIMIT_VALUE, EXCLUSIVE_UPPER_LIMIT)
        self.assertTrue(evaluation.nominal)


class BothLimitsEvaluatorTest(unittest.TestCase):
    """
    Tests for the both limits condition evaluator
    """
    def test_nominal_if_between_lower_and_upper(self):
        """
        Nominal if the measurement is between the lower and the upper limits
        """
        _, _, _, evaluation = evaluate_both_limits_condition(
            EXPECTED_VALUE,
            EXCLUSIVE_LOWER_LIMIT, EXCLUSIVE_UPPER_LIMIT)
        self.assertTrue(evaluation.nominal)
