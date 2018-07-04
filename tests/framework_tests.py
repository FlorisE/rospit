""" Tests for the PIT framework """
import unittest
from src.pit import TestSuite, \
                    TestCase, \
                    TestCaseReport, \
                    TestSuiteReport, \
                    BinaryCondition, \
                    BinarySensor, \
                    BinaryMeasurement, \
                    BinaryConditionEvaluator, \
                    InCategoriesCondition, \
                    CategorySensor, \
                    CategoryMeasurement, \
                    InCategoryConditionEvaluator, \
                    NumericSensor, \
                    NumericMeasurement, \
                    Limit, \
                    LowerLimitCondition, \
                    LowerLimitEvaluator, \
                    UpperLimitCondition, \
                    UpperLimitEvaluator, \
                    BothLimitsCondition, \
                    BothLimitsEvaluator, \
                    get_inclusive_limit, \
                    get_exclusive_limit


class MockTestCase(TestCase):
    """
    Mock for test cases
    """

    def __init__(self, name):
        TestCase.__init__(self, name)
        self.ran = False
        self.verify_preconditions_called = False
        self.verify_postconditions_called = False
        self.verify_invariants_called_at_ts = []

    def run(self):
        self.ran = True
        return TestCaseReport(self, [], [], [])

    def verify_preconditions(self):
        self.verify_preconditions_called = True
        return super(MockTestCase, self).verify_preconditions()

    def verify_postconditions(self):
        self.verify_postconditions_called = True
        return super(MockTestCase, self).verify_postconditions()

    def verify_invariants(self, t):
        self.verify_invariants_called_at_ts.append(t)
        return self.verify_invariants(t)


class TestTestSuite(unittest.TestCase):
    """
    Tests for the test suite class
    """
    def setUp(self):
        self.test_suite = TestSuite("Testing")

    def test_run_runs_all_test_cases(self):
        """
        Running the test suite should run all test cases
        """
        test_case_one = MockTestCase("Test case 1")
        test_case_two = MockTestCase("Test case 2")
        self.test_suite.test_cases.append(test_case_one)
        self.test_suite.test_cases.append(test_case_two)
        self.test_suite.run()
        self.assertTrue(test_case_one.ran)
        self.assertTrue(test_case_two.ran)

    def test_run_produces_reports(self):
        """
        Running the test suite should produce a report for each test case
        """
        test_case_one = MockTestCase("Test case 1")
        test_case_two = MockTestCase("Test case 2")
        self.test_suite.test_cases.append(test_case_one)
        self.test_suite.test_cases.append(test_case_two)
        report = self.test_suite.run()
        self.assertIsNotNone(report)


class TestTestCase(unittest.TestCase):
    """
    Tests for the test case class
    """
    def setUp(self):
        self.test_case = MockTestCase("Test case")

    def test_execute_verifies_and_runs(self):
        """
        Executing the test case should:
        1. Verify the preconditions
        2. Call the run method
        3. Verify the postconditions
        """
        self.test_case.execute()
        self.assertTrue(self.test_case.verify_preconditions_called)
        self.assertTrue(self.test_case.ran)
        self.assertTrue(self.test_case.verify_postconditions_called)


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


valid_category = "valid_category"


class ValidCategorySensor(CategorySensor):
    """
    Returns a valid category
    """
    def sense(self):
        return CategoryMeasurement(valid_category)


invalid_category = "invalid_category"


class InvalidCategorySensor(CategorySensor):
    """
    Returns an invalid category
    """
    def sense(self):
        return CategoryMeasurement(invalid_category)


def evaluate_category_condition(in_categories):
    """
    Evaluates a category condition
    """
    condition = InCategoriesCondition([valid_category])
    if in_categories:
        sensor = ValidCategorySensor()
    else:
        sensor = InvalidCategorySensor()
    evaluator = InCategoryConditionEvaluator(sensor)
    evaluation = evaluator.evaluate(condition)
    return (condition, sensor, evaluator, evaluation)


class InCategoryEvaluatorTest(unittest.TestCase):
    """
    Tests for in category evaluator
    """
    def test_evaluation_sets_the_condition(self):
        """
        Checks if the evaluator sets the condition
        """
        condition, _, _, evaluation = evaluate_category_condition(True)
        self.assertEqual(evaluation.condition, condition)

    def test_verifies_if_in_categories(self):
        """
        The evaluator should be nominal if the category is
        in the expected categories
        """
        _, _, _, evaluation = evaluate_category_condition(True)
        self.assertTrue(evaluation.nominal)

    def test_verifies_false_if_not_in_categories(self):
        """
        The evaluator should be nominal if the category is
        in the expected categories
        """
        _, _, _, evaluation = evaluate_category_condition(False)
        self.assertFalse(evaluation.nominal)


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
    def sense(self):
        return NumericMeasurement(BELOW_LOWER_LIMIT_VALUE)


class OnLowerLimitNumericSensor(NumericSensor):
    """
    Senses a value that is on the lower limit
    """
    def sense(self):
        return NumericMeasurement(LOWER_LIMIT_VALUE)


class BetweenLowerLimitAndExpectedValueNumericSensor(NumericSensor):
    """
    Senses a value that is above the lower limit but below the expected value
    """
    def sense(self):
        return NumericMeasurement(BETWEEN_LOWER_LIMIT_AND_EXPECTED_VALUE)


class ExpectedValueNumericSensor(NumericSensor):
    """
    Senses the expected value
    """
    def sense(self):
        return NumericMeasurement(EXPECTED_VALUE)


class BetweenExpectedValueAndUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is above the expected value but below the upper limit
    """
    def sense(self):
        return NumericMeasurement(BETWEEN_EXPECTED_AND_UPPER_LIMIT_VALUE)


class OnUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is on the upper limit
    """
    def sense(self):
        return NumericMeasurement(UPPER_LIMIT_VALUE)


class AboveUpperLimitNumericSensor(NumericSensor):
    """
    Senses a value that is above the upper limit
    """
    def sense(self):
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
