""" Tests for binary evaluators """
import unittest
from rospit.binary import BinaryCondition, \
                          BinarySensor, \
                          BinaryMeasurement, \
                          BinaryConditionEvaluator


class AlwaysTrueBinarySensor(BinarySensor):
    """
    Always returns True
    """
    def sense(self):
        return BinaryMeasurement(True)


class AlwaysFalseBinarySensor(BinarySensor):
    """
    Always returns True
    """
    def sense(self):
        return BinaryMeasurement(False)


def evaluate_binary_condition(expected, actual):
    """
    Evaluates a binary condition
    """
    condition = BinaryCondition(expected)
    if actual:
        sensor = AlwaysTrueBinarySensor()
    else:
        sensor = AlwaysFalseBinarySensor()
    evaluator = BinaryConditionEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


class BinaryEvaluatorTest(unittest.TestCase):
    """
    Tests for binary evaluator
    """

    def test_evaluation_sets_the_condition(self):
        """
        Checks if the evaluator sets the condition
        """
        condition, _, _, evaluation = evaluate_binary_condition(True, True)
        self.assertEqual(evaluation.condition, condition)

    def test_verifies_if_expected_true_and_measured_true(self):
        """
        The evaluator should be nominal if true was expected and also measured
        """
        _, _, _, evaluation = evaluate_binary_condition(True, True)
        self.assertTrue(evaluation.nominal)

    def test_verifies_if_expected_false_and_measured_false(self):
        """
        The evaluator should be nominal if true was expected and also measured
        """
        _, _, _, evaluation = evaluate_binary_condition(False, False)
        self.assertTrue(evaluation.nominal)

    def test_verifies_false_if_expected_false_and_measured_true(self):
        """
        The evaluator should not be nominal if true was expected
        but not measured
        """
        _, _, _, evaluation = evaluate_binary_condition(False, True)
        self.assertFalse(evaluation.nominal)

    def test_verifies_false_if_expected_true_and_measured_false(self):
        """
        The evaluator should not be nominal if true was expected
        but not measured
        """
        _, _, _, evaluation = evaluate_binary_condition(True, False)
        self.assertFalse(evaluation.nominal)
