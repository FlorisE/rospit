"""
Script that launches useful nodes and services for executing Physical Integration Tests
"""
import logging
import sys
import rospy
from rospit.framework import test_runner
from rospit_msgs.srv import ExecuteXMLTestSuite, GetAllReports
from rospit_msgs.msg import TestSuiteReport as TestSuiteReportMessage
from rospit_xml import get_test_suite_from_xml_path


class TestRunnerNode(object):
    """
    A node that runs tests and publishers their results
    """
    def __init__(self):
        rospy.init_node("test_runner", anonymous=True)
        self.execution_service = rospy.Service(
            "execute_xml_test_suite", ExecuteXMLTestSuite, self.execute_xml_test_suite)
        self.reports_service = rospy.Service(
            "get_all_reports", GetAllReports, self.get_all_reports)
        self.test_suite_report_publisher = rospy.Publisher(
            "test_suite_reports", TestSuiteReportMessage)
        self.spinning = False
        self.reports = []

    def execute_xml_test_suite(self, request):
        """
        Execute a test suite specified in an XML file.
        Request should be a string specifying the path to the test to run.
        """
        parser = get_test_suite_from_xml_path(request.path.data, True)
        logger = logging.getLogger("rospit")
        logger.setLevel(logging.INFO)
        logger.addHandler(logging.StreamHandler(sys.stdout))
        test_suite = parser.parse()
        report = test_suite.run(logger)
        self.test_suite_report_publisher.publish(report)
        mapped_report = test_runner.map_test_suite_report(report)
        self.reports.append(mapped_report)
        return mapped_report

    def get_all_reports(self):
        """
        Returns all the reports that have been generated since this service has been active
        """
        return self.reports

    def spin(self):
        """ Spins the node """
        self.spinning = True
        rospy.spin()


if __name__ == '__main__':
    TestRunnerNode()
    rospy.spin()
