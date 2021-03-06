from std_msgs.msg import String as StringMessage, \
                         Bool as BoolMessage
from rospit_msgs.msg import TestSuiteReport as TestSuiteReportMessage, \
                            TestCaseReport as TestCaseReportMessage, \
                            TestCase as TestCaseMessage, \
                            Condition as ConditionMessage, \
                            Evaluation as EvaluationMessage, \
                            ConditionEvaluationPair as ConditionEvaluationPairMessage


def map_evaluation(evaluation):
    """
    Maps an evaluation from the framework to a ROS ConditionEvaluationPair
    """
    condition_msg = ConditionMessage(name=StringMessage(evaluation.condition.name))
    evaluation_msg = EvaluationMessage(
        nominal=BoolMessage(evaluation.nominal),
        payload=StringMessage(evaluation.expected_actual_string()))
    return ConditionEvaluationPairMessage(condition=condition_msg, evaluation=evaluation_msg)


def map_test_case_report(report):
    """
    Maps a test case report from the framework to a ROS message
    """
    tc = TestCaseMessage(StringMessage(report.test_case.name))
    pre_c = [map_evaluation(condition) for condition in report.preconditions]
    iv = [map_evaluation(condition) for condition in report.invariants]
    post_c = [map_evaluation(condition) for condition in report.postconditions]
    return TestCaseReportMessage(
        test_case=tc, preconditions=pre_c, invariants=iv, postconditions=post_c)


def map_test_suite_report(report):
    """
    Maps a report from the test framework to a ROS message
    """
    reports = [map_test_case_report(tcr) for tcr in report.test_case_reports]
    return TestSuiteReportMessage(
        test_suite_name=StringMessage(report.test_suite.name), test_case_reports=reports)
