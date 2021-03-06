""" Tests for the PIT framework """

import unittest
from rospit.framework import TestSuite, \
                             TestCase, \
                             TestCaseReport, \
                             Evaluation, \
                             Measurement, \
                             Condition


class LoggerMock(object):
    def info(self, string):
        pass

    def debug(self, string):
        pass


class MockTestCase(TestCase):
    """
    Mock for test cases
    """

    def __init__(self, name):
        TestCase.__init__(self, name)
        self.ran = False
        self.verify_preconditions_called = False
        self.verify_postconditions_called = False
        self.verify_invariants_called = False
        self.set_up_called = False
        self.tear_down_called = False

    def run(self):
        self.ran = True
        return TestCaseReport(self, [], [], [], [])

    def set_up(self):
        self.set_up_called = True
        return super(MockTestCase, self).set_up()

    def verify_preconditions(self):
        self.verify_preconditions_called = True
        return super(MockTestCase, self).verify_preconditions()

    def verify_postconditions(self):
        self.verify_postconditions_called = True
        return super(MockTestCase, self).verify_postconditions()

    def verify_invariants(self):
        self.verify_invariants_called = True
        return super(MockTestCase, self).verify_invariants()

    def tear_down(self):
        self.tear_down_called = True
        return super(MockTestCase, self).tear_down()


class TestCaseWithFailedPreconditions(TestCase):
    def __init__(self):
        TestCase.__init__(self, "Test case with failed preconditions")

    def run(self):
        pass

    def execute(self, logger=None):
        return TestCaseReport(
            self,
            [Evaluation(Measurement(0), Condition(None, "Failed condition"), False)],
            [], [], [])


class TestTestSuite(unittest.TestCase):
    """
    Tests for the test suite class
    """
    def setUp(self):
        self.test_suite = TestSuite("Testing")
        self.logger = LoggerMock()

    def test_run_runs_all_test_cases(self):
        """
        Running the test suite should run all test cases
        """
        test_case_one = MockTestCase("Test case 1")
        test_case_two = MockTestCase("Test case 2")
        self.test_suite.test_cases.append(test_case_one)
        self.test_suite.test_cases.append(test_case_two)
        self.test_suite.run(self.logger)
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
        report = self.test_suite.run(self.logger)
        self.assertIsNotNone(report)

    def test_get_junit_xml(self):
        test_case_one = MockTestCase("Test case 1")
        test_case_two = MockTestCase("Test case 2")
        self.test_suite.test_cases.append(test_case_one)
        self.test_suite.test_cases.append(test_case_two)
        report = self.test_suite.run(self.logger)
        self.assertEqual('''\
<testsuite name="Testing" tests="2">
<testcase classname="Test case 1" name="Test case 1" />
<testcase classname="Test case 2" name="Test case 2" />
</testsuite>''', report.get_junit_xml())

    def test_test_case_with_failed_preconditions(self):
        test_case = TestCaseWithFailedPreconditions()
        self.test_suite.test_cases.append(test_case)
        report = self.test_suite.run(self.logger)
        self.assertEqual('''\
<testsuite name="Testing" tests="1">
<testcase classname="Test case with failed preconditions" name="Test case with failed preconditions">
<failure type="preconditions">Failed condition</failure>
</testcase>
</testsuite>''', report.get_junit_xml())


class TestTestCase(unittest.TestCase):
    """
    Tests for the test case class
    """
    def setUp(self):
        self.test_case = MockTestCase("Test case")
        self.logger = LoggerMock()

    def test_execute_verifies_and_runs(self):
        """
        Executing the test case should:
        1. Set up
        2. Verify the preconditions
        3. Call the run method
        4. Verify the postconditions
        5. Tear down
        """
        self.test_case.execute(self.logger)
        self.assertTrue(self.test_case.set_up_called)
        self.assertTrue(self.test_case.verify_preconditions_called)
        self.assertTrue(self.test_case.ran)
        self.assertTrue(self.test_case.verify_postconditions_called)
        self.assertTrue(self.test_case.tear_down_called)
