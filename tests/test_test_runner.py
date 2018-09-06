""" Tests for the PIT test runner """

import unittest
from rospit.framework import Condition, Evaluation, Measurement
from rospit.framework import TestSuite as PITTestSuite, \
                             TestSuiteReport as PITTestSuiteReport, \
                             TestCaseReport as PITTestCaseReport, \
                             TestCase as PITTestCase
import rospit.test_runner


class _TestCaseMock(PITTestCase):
    def __init__(self):
        PITTestCase.__init__(self, "test")

    def run(self): pass


class TestMapper(unittest.TestCase):
    """ Tests the mapping functions """

    def test_maps_test_suite_report(self):
        """ Tests the mapping function for an entire test suite report """
        test_suite = PITTestSuite("test")
        in_report = PITTestSuiteReport(test_suite, [])
        out_report = rospit.test_runner.map_test_suite_report(in_report)
        self.assertEqual("test", out_report.test_suite_name.data)

    def test_maps_test_case_report(self):
        """ Tests the mapping function for a single test case report """
        tc = _TestCaseMock()
        in_report = PITTestCaseReport(tc, None, None, None, None)
        out_report = rospit.test_runner.map_test_case_report(in_report)
        self.assertEqual("test", out_report.test_case.name.data)

    def test_maps_evaluation(self):
        """ Tests the mapping function for an evaluation """
        condition = Condition(None, "NoneCondition")
        measurement = Measurement(None)
        in_evaluation = Evaluation(measurement, condition, True)
        out_evaluation = rospit.test_runner.map_evaluation(in_evaluation)
        self.assertEqual("NoneCondition", out_evaluation.condition.name.data)
