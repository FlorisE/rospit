""" A framework for running automated tests using external instrumentation """

from abc import ABCMeta, abstractmethod
from collections import namedtuple
import future  # noqa:401, pylint: disable=W0611


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
        return test_suite.run()

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
        test_case_reports = [tc.execute() for tc in self.test_cases]
        return TestSuiteReport(test_case_reports)


TestCaseReport = namedtuple(
    "TestCaseReport",
    ["test_case", "preconditions", "invariants", "postconditions"])


TestSuiteReport = namedtuple(
    "TestSuiteReport",
    ["test_case_reports"])


class TestCase(object):
    """
    Specifies a test case with preconditions, invariants and postconditions
    """
    __metaclass__ = ABCMeta

    def __init__(self, name="", preconditions=None, invariants=None,
                 postconditions=None):
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions

    def verify_preconditions(self):
        """
        Evaluates all preconditions
        """
        return [c.evaluate() for c in self.preconditions]

    def verify_invariants(self, time):
        """
        Evaluates the invarians at time since starting
        """
        return [c.evaluate(time) for c in self.invariants]

    def verify_postconditions(self):
        """
        Evaluates all postconditions
        """
        return [c.evaluate() for c in self.postconditions]

    def execute(self):
        """
        Verifies the preconditions, runs the test, verifies the postconditions
        """
        preconditions_evaluation = self.verify_preconditions()
        invariants = self.run()
        postconditions_evaluation = self.verify_postconditions()
        return TestCaseReport(self, preconditions_evaluation, invariants,
                              postconditions_evaluation)

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

    def __init__(self, name):
        self.name = name


class Evaluator(object):
    """
    Abstract base class for evaluators
    """
    __metaclass__ = ABCMeta

    def __init__(self, sensor):
        self.sensor = sensor

    @abstractmethod
    def evaluate(self, condition, measurement):
        """
        Executes the evaluator by checking the measurement using the condition
        """
        pass


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
        pass

    @abstractmethod
    def sense(self):
        """
        Reads the sensor value
        """
        pass
