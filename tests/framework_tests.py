""" Tests for the PIT framework """
import unittest
from src.pit import TestSuite, TestCase, Report, InCategoryCondition, \
                    BinaryCondition, NumericCondition


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
        return Report(self, True, [], [], [])

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
        reports = self.test_suite.run()
        self.assertEqual(len(reports), 2)


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


class ConcreteInCategoryCondition(InCategoryCondition):
    """
    Test implementation for InCategoryCondition
    """
    VALID_CAT_FROM = 0
    INVALID_CAT_FROM = 2

    def __init__(self, name):
        InCategoryCondition.__init__(self, name)
        self.categories = set(["cat 1", "cat 2"])

    def measure_at(self, t):
        if t >= 0 and t < 1:
            return "cat 1"
        elif t >= 1 and t < 2:
            return "cat 2"
        else:
            return "cat 3"


class InCategoryConditionTests(unittest.TestCase):
    """
    Tests for the InCategoryCondition class
    """
    def setUp(self):
        self.condition = ConcreteInCategoryCondition("Test condition")

    def test_verifies_true_if_valid(self):
        """
        If the verifier is valid, return True
        """
        self.assertTrue(
            self.condition.verify(
                ConcreteInCategoryCondition.VALID_CAT_FROM))

    def test_verifies_false_if_invalid(self):
        """
        If the verifier is invalid, return False
        """
        self.assertFalse(
            self.condition.verify(
                ConcreteInCategoryCondition.INVALID_CAT_FROM))


class ConcreteBinaryCondition(BinaryCondition):
    '''
    Verifies correct at t == 0 and t == 3
    Verifies incorrect at t == 1 and t == 2
    '''
    EXPECT_FALSE_AND_MEASURE_FALSE_AT = 0
    EXPECT_FALSE_AND_MEASURE_TRUE_AT = 1
    EXPECT_TRUE_AND_MEASURE_FALSE_AT = 2
    EXPECT_TRUE_AND_MEASURE_TRUE_AT = 3
    EXPECT_FALSE_AT = [EXPECT_FALSE_AND_MEASURE_FALSE_AT,
                       EXPECT_FALSE_AND_MEASURE_TRUE_AT]
    EXPECT_TRUE_AT = [EXPECT_TRUE_AND_MEASURE_FALSE_AT,
                      EXPECT_TRUE_AND_MEASURE_TRUE_AT]
    MEASURE_FALSE_AT = [EXPECT_FALSE_AND_MEASURE_FALSE_AT,
                        EXPECT_TRUE_AND_MEASURE_FALSE_AT]
    MEASURE_TRUE_AT = [EXPECT_FALSE_AND_MEASURE_TRUE_AT,
                       EXPECT_TRUE_AND_MEASURE_TRUE_AT]

    def __init__(self, name):
        BinaryCondition.__init__(self, name)

    def expect_at(self, time):
        if time in ConcreteBinaryCondition.EXPECT_FALSE_AT:
            return False
        elif time in ConcreteBinaryCondition.EXPECT_TRUE_AT:
            return True
        else:
            raise Exception("Invalid input for test")

    def measure_at(self, time):
        if time in ConcreteBinaryCondition.MEASURE_FALSE_AT:
            return False
        elif time in ConcreteBinaryCondition.MEASURE_TRUE_AT:
            return True
        else:
            raise Exception("Invalid input for test")


class BinaryConditionTests(unittest.TestCase):
    """
    Tets for the BinaryCondition class
    """
    def setUp(self):
        self.condition = ConcreteBinaryCondition("Test condition")

    def test_verifies_if_expect_and_measure_both_true(self):
        """
        In this case the condition is valid because we measured true and
        expected true.
        """
        self.assertTrue(
            self.condition.verify(
                ConcreteBinaryCondition.EXPECT_TRUE_AND_MEASURE_TRUE_AT))

    def test_verifies_if_expect_and_measure_both_false(self):
        """
        In this case the condition is valid because we measured false and
        expected false.
        """
        self.assertTrue(
            self.condition.verify(
                ConcreteBinaryCondition.EXPECT_FALSE_AND_MEASURE_FALSE_AT))

    def test_verifies_false_if_expect_false_and_measure_true(self):
        """
        In this case the condition is invalid because we measured true but
        expected false.
        """
        self.assertFalse(
            self.condition.verify(
                ConcreteBinaryCondition.EXPECT_FALSE_AND_MEASURE_TRUE_AT))

    def test_verifies_false_if_expect_true_and_measure_false(self):
        """
        In this case the condition is invalid because we measured false but
        expected true
        """
        self.assertFalse(
            self.condition.verify(
                ConcreteBinaryCondition.EXPECT_TRUE_AND_MEASURE_FALSE_AT))


class ConcreteNumericCondition(NumericCondition):
    """
    Test implementation for the NumericCondition class
    For all time, expected value is 0, min is -2, max is 2
    """
    # defining interesting values
    EXPECTED_VALUE = 0
    LOWER_LIMIT_VALUE = -2
    UPPER_LIMIT_VALUE = 2

    # definding interesting times
    EXACT_MATCH_AT = 1
    HIGHER_THAN_UPPER_LIMIT_AT = 2
    LOWER_THAN_LOWER_LIMIT_AT = 3
    ON_UPPER_LIMIT_AT = 4
    ON_LOWER_LIMIT_AT = 5
    HIGHER_BUT_ACCEPTABLE_AT = 6
    LOWER_BUT_ACCEPTABLE_AT = 7

    def __init__(
            self, name, lower_limit_is_inclusive, upper_limit_is_inclusive):
        NumericCondition.__init__(
            self, name, lower_limit_is_inclusive, upper_limit_is_inclusive)

    def measure_at(self, time):
        CNC = ConcreteNumericCondition
        return {
            CNC.EXACT_MATCH_AT: CNC.EXPECTED_VALUE,
            CNC.HIGHER_THAN_UPPER_LIMIT_AT: CNC.UPPER_LIMIT_VALUE+1,
            CNC.LOWER_THAN_LOWER_LIMIT_AT: CNC.LOWER_LIMIT_VALUE-1,
            CNC.ON_UPPER_LIMIT_AT: CNC.UPPER_LIMIT_VALUE,
            CNC.ON_LOWER_LIMIT_AT: CNC.LOWER_LIMIT_VALUE,
            CNC.HIGHER_BUT_ACCEPTABLE_AT: CNC.EXPECTED_VALUE+1,
            CNC.LOWER_BUT_ACCEPTABLE_AT: CNC.EXPECTED_VALUE-1,
        }[time]

    def upper_limit_at(self, time):
        return ConcreteNumericCondition.UPPER_LIMIT_VALUE

    def lower_limit_at(self, time):
        return ConcreteNumericCondition.LOWER_LIMIT_VALUE


class InclusiveConcreteNumericCondition(ConcreteNumericCondition):
    """
    Test implementation for the NumericCondition class
    Using least_is_inclusive = most_is_inclusive = True
    For all time, expected value is 0, min is -2, max is 2
    """
    VALIDATE_FALSE_AT = [
        ConcreteNumericCondition.HIGHER_THAN_UPPER_LIMIT_AT,
        ConcreteNumericCondition.LOWER_THAN_LOWER_LIMIT_AT
    ]
    VALIDATE_TRUE_AT = [
        ConcreteNumericCondition.EXACT_MATCH_AT,
        ConcreteNumericCondition.ON_UPPER_LIMIT_AT,
        ConcreteNumericCondition.ON_LOWER_LIMIT_AT,
        ConcreteNumericCondition.HIGHER_BUT_ACCEPTABLE_AT,
        ConcreteNumericCondition.LOWER_BUT_ACCEPTABLE_AT
    ]

    def __init__(self, name):
        ConcreteNumericCondition.__init__(self, name, True, True)


class InclusiveNumericConditionTests(unittest.TestCase):
    """
    Tests for the NumericCondition class
    """
    def setUp(self):
        self.condition = InclusiveConcreteNumericCondition("Test condition")

    def verifier_helper(self, condition):
        """
        A helper function to verify whether the expected test value
        matches the measured value
        """
        condition_result = self.condition.verify(condition)
        if condition in InclusiveConcreteNumericCondition.VALIDATE_TRUE_AT:
            return self.assertTrue(condition_result)
        elif condition in InclusiveConcreteNumericCondition.VALIDATE_FALSE_AT:
            return self.assertFalse(condition_result)
        else:
            raise Exception("Invalid input for test")

    def test_verifies_if_exact_match(self):
        """
        In this case the condition is valid because we have an exact match
        """
        self.verifier_helper(
            ConcreteNumericCondition.EXACT_MATCH_AT)

    def test_verifies_false_if_higher_than_upper_limit(self):
        """
        In this case the condition is invalid because the measured value is
        higher than the upper limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.HIGHER_THAN_UPPER_LIMIT_AT)

    def test_verifies_false_if_lower_than_lower_limit(self):
        """
        In this case the condition is invalid because the measured value is
        lower than the lower limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.LOWER_THAN_LOWER_LIMIT_AT)

    def test_verifies_if_on_upper_limit(self):
        """
        In this case the condition is valid because the measured value is
        on the upper limit, and the upper limit is inclusive
        """
        self.verifier_helper(
            ConcreteNumericCondition.ON_UPPER_LIMIT_AT)

    def test_verifies_if_on_lower_limit(self):
        """
        In this case the condition is valid because the measured value is
        on the lower limit, and the lower limit is inclusive
        """
        self.verifier_helper(
            ConcreteNumericCondition.ON_LOWER_LIMIT_AT)

    def test_verifies_if_within_upper_limit(self):
        """
        In this case the condition is valid because the measured value is
        between the expected value and the upper limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.HIGHER_BUT_ACCEPTABLE_AT)

    def test_verifies_if_within_lower_limit(self):
        """
        In this case the condition is valid because the measured value is
        between the expected value and the lower limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.LOWER_BUT_ACCEPTABLE_AT)


class ExclusiveConcreteNumericCondition(ConcreteNumericCondition):
    """
    Test implementation for the NumericCondition class
    Using least_is_inclusive = most_is_inclusive = False
    """
    VALIDATE_FALSE_AT = [
        ConcreteNumericCondition.HIGHER_THAN_UPPER_LIMIT_AT,
        ConcreteNumericCondition.LOWER_THAN_LOWER_LIMIT_AT,
        ConcreteNumericCondition.ON_UPPER_LIMIT_AT,
        ConcreteNumericCondition.ON_LOWER_LIMIT_AT
    ]
    VALIDATE_TRUE_AT = [
        ConcreteNumericCondition.EXACT_MATCH_AT,
        ConcreteNumericCondition.HIGHER_BUT_ACCEPTABLE_AT,
        ConcreteNumericCondition.LOWER_BUT_ACCEPTABLE_AT
    ]

    def __init__(self, name):
        ConcreteNumericCondition.__init__(self, name, False, False)


class ExclusiveNumericConditionTests(unittest.TestCase):
    """
    Tests for the NumericCondition class
    """
    def setUp(self):
        self.condition = ExclusiveConcreteNumericCondition("Test condition")

    def verifier_helper(self, condition):
        """
        A helper function to verify whether the expected test value
        matches the measured value
        """
        condition_result = self.condition.verify(condition)
        if condition in ExclusiveConcreteNumericCondition.VALIDATE_TRUE_AT:
            return self.assertTrue(condition_result)
        elif condition in ExclusiveConcreteNumericCondition.VALIDATE_FALSE_AT:
            return self.assertFalse(condition_result)
        else:
            raise Exception("Invalid input for test")

    def test_verifies_if_exact_match(self):
        """
        In this case the condition is valid because we have an exact match
        """
        self.verifier_helper(
            ConcreteNumericCondition.EXACT_MATCH_AT)

    def test_verifies_false_if_higher_than_upper_limit(self):
        """
        In this case the condition is invalid because the measured value is
        higher than the upper limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.HIGHER_THAN_UPPER_LIMIT_AT)

    def test_verifies_false_if_lower_than_lower_limit(self):
        """
        In this case the condition is invalid because the measured value is
        lower than the lower limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.LOWER_THAN_LOWER_LIMIT_AT)

    def test_verifies_false_if_on_upper_limit(self):
        """
        In this case the condition is valid because the measured value is
        on the upper limit, and the upper limit is inclusive
        """
        self.verifier_helper(
            ConcreteNumericCondition.ON_UPPER_LIMIT_AT)

    def test_verifies_false_if_on_lower_limit(self):
        """
        In this case the condition is valid because the measured value is
        on the lower limit, and the lower limit is inclusive
        """
        self.verifier_helper(
            ConcreteNumericCondition.ON_LOWER_LIMIT_AT)

    def test_verifies_if_within_upper_limit(self):
        """
        In this case the condition is valid because the measured value is
        between the expected value and the upper limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.HIGHER_BUT_ACCEPTABLE_AT)

    def test_verifies_if_within_lower_limit(self):
        """
        In this case the condition is valid because the measured value is
        between the expected value and the lower limit
        """
        self.verifier_helper(
            ConcreteNumericCondition.LOWER_BUT_ACCEPTABLE_AT)
