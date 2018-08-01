""" A framework for running automated tests using external instrumentation """

from abc import ABCMeta, abstractmethod
from collections import namedtuple
from time import sleep
import future  # noqa:401, pylint: disable=W0611
from logging import getLogger


logger = getLogger("rospit framework")


class StatusMessages(object):
    PASSED = '\033[92m' + "PASSED" + '\033[0m'
    FAILED = '\033[91m' + "FAILED" + '\033[0m'


def status_message_from_boolean(value):
    if value:
        return StatusMessages.PASSED
    else:
        return StatusMessages.FAILED


class Runner(object):
    """
    Class for running test suites
    """

    def __init__(self):
        self.reports = []
        self.last_suite = None

    def run_suite(self, test_suite):
        """
        Runs the specified test_suite
        """
        self.last_suite = test_suite
        test_suite_report = test_suite.run()
        for test_case_report in test_suite_report.test_case_reports:
            status_message = status_message_from_boolean(test_case_report.passed())
            logger.info("{}: {}".format(test_case_report.test_case.name, status_message))

    def run_all(self):
        """
        Autodetect test suites and runs them
        """
        pass


class TestSuite(object):
    """
    Container for test cases
    """
    __metaclass__ = ABCMeta

    def __init__(self, name=""):
        self.name = name
        self.test_cases = []

    def run(self):
        """
        Runs all the test cases in this test suite
        """
        logger.info("Running test suite {}".format(self.name))
        test_case_reports = [tc.execute() for tc in self.test_cases]
        return TestSuiteReport(self, test_case_reports)



class TestCaseReport(object):
    def __init__(self, test_case, preconditions, invariants, postconditions, not_passed_dependencies):
        self.test_case = test_case
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions
        self.not_passed_dependencies = not_passed_dependencies
        self.preconditions_nominal = all_conditions_nominal(self.preconditions)
        self.invariants_nominal = all_conditions_nominal(self.invariants)
        self.postconditions_nominal = all_conditions_nominal(self.postconditions)
        self.passed = self.preconditions_nominal and \
                      self.invariants_nominal and \
                      self.postconditions_nominal and \
                      len(self.not_passed_dependencies) == 0
        self.failure_type = None if self.passed else \
                            "dependencies" if not len(self.not_passed_dependencies) == 0 else \
                            "preconditions" if not self.preconditions_nominal else \
                            "invariants" if not self.invariants_nominal else \
                            "postconditions" if not self.postconditions_nominal else ""

    def get_failure(self):
        if self.passed:
            return ""
        if self.failure_type == "dependencies":
            failure = ", ".join([tc.name if tc.name != "" else "UNKNOWN" for tc in self.not_passed_dependencies])
        elif self.failure_type == "preconditions":
            failure = get_failed_conditions(self.preconditions)
        elif self.failure_type == "invariants":
            failure = get_failed_conditions(self.invariants)
        else:
            failure = get_failed_conditions(self.postconditions)
        return '''<failure type="{}">{}</failure>'''.format(self.failure_type, failure)


    def get_junit_xml(self):
        if self.passed:
            return '''\
<testcase classname="{}" name="{}" />'''.format(self.test_case.name, self.test_case.name)
        else:
            return '''\
<testcase classname="{}" name="{}">
{}
</testcase>'''.format(self.test_case.name, self.test_case.name, self.get_failure())



class TestSuiteReport(object):
    def __init__(self, test_suite, test_case_reports):
        self.test_suite = test_suite
        self.test_case_reports = test_case_reports

    def get_junit_xml(self):
        return '''\
<testsuite name="{}" tests="{}">
{}
</testsuite>'''.format(self.test_suite.name if self.test_suite is not None else "UNKNOWN", \
                       len(self.test_case_reports), "\n".join(
                           [test_case_report.get_junit_xml() for test_case_report in self.test_case_reports]))

def get_failed_conditions(conditions):
    if conditions is None:
        return ""
    return ", ".join([c.condition.name for c in conditions if not c.nominal])


def all_conditions_nominal(conditions):
    if conditions is None:
        return True
    return all([c.nominal for c in conditions])


class TestCase(object):
    """
    Specifies a test case with preconditions, invariants and postconditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name="", preconditions=None, invariants=None,
                 postconditions=None, wait_for_preconditions=False,
                 sleep_rate=0.1, depends_on=None):
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        if depends_on is None:
            depends_on = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions
        self.wait_for_preconditions = wait_for_preconditions
        self.sleep_rate = sleep_rate
        self.depends_on = depends_on
        self.report = None
        self.ran = False

    def verify_preconditions(self):
        """
        Evaluates all preconditions
        """
        return [e.evaluate(c) for c, e in self.preconditions]

    def verify_invariants(self):
        """
        Evaluates the invarians since starting
        """
        return [e.evaluate(c) for c, e in self.invariants]

    def verify_postconditions(self):
        """
        Evaluates all postconditions
        """
        return [e.evaluate(c) for c, e in self.postconditions]

    def all_preconditions_nominal(self, preconditions=None):
        if preconditions is None:
            preconditions = self.verify_preconditions()
        return all_conditions_nominal(preconditions)

    def execute(self):
        """
        Verifies the preconditions, runs the test, verifies the postconditions
        """
        logger.info("Running test case {}".format(self.name))

        not_passed_dependencies = [tc for tc in self.depends_on if not tc.ran() or not tc.report.passed()]

        if len(not_passed_dependencies) > 0:
            logger.info("Dependencies have not been met, marking test case as failure")
            self.finish([], [], [], not_passed_dependencies)
            return self.report

        logger.info("Verifying preconditions")
        if self.wait_for_preconditions:
            logger.info("Wait for preconditions is enabled")
            preconditions_evaluation = self.verify_preconditions()
            while not self.all_preconditions_nominal(preconditions_evaluation):
                sleep(self.sleep_rate)
                preconditions_evaluation = self.verify_preconditions()
        else:
            logger.info("Wait for preconditions is disabled")
            preconditions_evaluation = self.verify_preconditions()
            if not self.all_preconditions_nominal(preconditions_evaluation):
                self.finish(preconditions_evaluation, [], [])
                return self.report
        logger.info("Running the body of the test")
        self.run()
        invariants_evaluation = self.verify_invariants()
        logger.info("Verifying postconditions")
        postconditions_evaluation = self.verify_postconditions()
        self.finish(preconditions_evaluation, invariants_evaluation, postconditions_evaluation)
        return self.report

    def finish(self, preconditions_evaluation, invariants, postconditions_evaluation, not_passed_dependencies=None):
        if not_passed_dependencies is None:
            not_passed_dependencies = []
        self.ran = True
        self.report = TestCaseReport(self, preconditions_evaluation, invariants,
                                     postconditions_evaluation, not_passed_dependencies)
        return self.report

    @abstractmethod
    def run(self):
        """
        Runs the test
        """
        pass


class Evaluation(object):
    """
    Evaluation produced by an Evaluator
    """
    def __init__(self, measurement, condition, nominal):
        self.measurement = measurement
        self.condition = condition
        self.nominal = nominal

    def get_failure(self):
        if self.nominal:
            return None
        else:
            return self.condition.name


class CompositeEvaluation(Evaluation):
    """
    Evaluation which consists of a list of evaluations for a single measurement
    """
    def __init__(self, measurement, condition, evaluations):
        Evaluation.__init__(
            self, measurement, condition,
            all([evaluation.nominal for evaluation in evaluations]))
        self.evaluations = evaluations


class TimestampEvaluationPair(object):
    """
    Pair of a timestamp and an evaluation
    """
    def __init__(self, timestamp, evaluation):
        self.timestamp = timestamp
        self.evaluation = evaluation


class ConditionEvaluatorPair(object):
    """
    Pair of a condition and an evaluator
    """
    def __init__(self, condition, evaluator):
        self.condition = condition
        self.evaluator = evaluator


class Condition(object):
    """
    Abstract base class for conditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, value, name=""):
        self.value = value
        self.name = name


class Evaluator(object):
    """
    Abstract base class for evaluators
    """
    __metaclass__ = ABCMeta

    def __init__(self, evaluator):
        self.evaluator = evaluator

    @abstractmethod
    def evaluate(self, condition, measurement):
        """
        Executes the evaluator by checking the measurement using the condition
        """
        pass

    def call_evaluator(self):
        if isinstance(self.evaluator, Sensor):
            return self.evaluator.sense()
        elif callable(self.evaluator):
            return self.evaluator()
        else:
            raise Exception("Evaluator is neither a sensor nor callable")


class Measurement(object):
    """
    Contains a measurement
    """
    __metaclass__ = ABCMeta


class Sensor(object):
    """
    Abstract base class for sensors
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        self.last_sensed = None

    @abstractmethod
    def sense_internal(self):
        """
        Reads the sensor value
        """
        pass

    def sense(self):
        self.last_sensed = self.sense_internal()
        return self.last_sensed
