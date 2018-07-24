""" Tests for the PIT framework """

import unittest

from rospit.framework import TestSuite, \
                             TestCase, \
                             TestCaseReport


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
