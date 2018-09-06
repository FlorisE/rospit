import unittest
import rospit.rospit_xml
from rospit.framework import Condition, Evaluator
from rospit.binary import BinaryCondition, StaticBooleanEvaluator
from rospit.declarative import DummyStep
import os
from lxml.etree import XMLSyntaxError

test_xml = '''\
<TestSuite xmlns='https://www.aist.go.jp/rospit' xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" name="TS">
<TestCase name="TC1">
<SetUp>
<Step xsi:type="Dummy" />
</SetUp>
<Preconditions>
<Precondition>
<Condition xsi:type="Binary" name="TrueCondition" value="true" />
<Evaluator xsi:type="StaticBooleanEvaluator" value="true" />
</Precondition>
</Preconditions>
<Run>
<Step xsi:type="Dummy" />
</Run>
<Postconditions>
<Postcondition>
<Condition xsi:type="Binary" name="FalseCondition" value="false" />
<Evaluator xsi:type="StaticBooleanEvaluator" value="false" />
</Postcondition>
</Postconditions>
<TearDown>
<Step xsi:type="Dummy" />
</TearDown>
</TestCase>
<TestCase name="TC2">
<Run />
</TestCase>
</TestSuite>'''

incorrect_xml = ""


class TestLoadTestSuite(unittest.TestCase):
    def test_parse_correct_xml_does_not_throw(self):
        rospit.rospit_xml.get_test_suite_from_xml_string(test_xml)

    def test_parse_incorrect_xml_throws(self):
        with self.assertRaises(XMLSyntaxError):
            rospit.rospit_xml.get_test_suite_from_xml_string(incorrect_xml)


class TestParseTestSuite(unittest.TestCase):
    def setUp(self):
        parser = rospit.rospit_xml.get_test_suite_from_xml_string(test_xml)
        parser.parse()
        self.test_suite = parser.test_suite

    def test_test_suite_name_set(self):
        self.assertEquals(self.test_suite.name, "TS")

    def test_test_cases_added(self):
        self.assertEquals(len(self.test_suite.test_cases), 2)


class TestParseFirstTestCase(unittest.TestCase):
    def setUp(self):
        parser = rospit.rospit_xml.get_test_suite_from_xml_string(test_xml)
        parser.parse()
        self.test_suite = parser.test_suite
        self.first_test_case = self.test_suite.test_cases[0]

    def test_test_case_1(self):
        self.assertEquals(self.first_test_case.name, "TC1")

    def test_test_case_1_set_up(self):
        self.assertEquals(len(self.first_test_case.set_up_steps), 1)
        step = self.first_test_case.set_up_steps[0]
        self.assertIsInstance(step, DummyStep)

    def test_test_case_1_preconditions(self):
        self.assertEquals(len(self.first_test_case.preconditions), 1)
        condition_evaluator_pair = self.first_test_case.preconditions[0]
        self.assertIsInstance(condition_evaluator_pair.condition, BinaryCondition)
        self.assertEquals(condition_evaluator_pair.condition.name, "TrueCondition") 
        self.assertTrue(condition_evaluator_pair.condition.value) 
        self.assertIsInstance(condition_evaluator_pair.evaluator, StaticBooleanEvaluator)
        self.assertTrue(condition_evaluator_pair.evaluator.always_true)

    def test_test_case_1_run(self):
        self.assertEquals(len(self.first_test_case.run_steps), 1)
        step = self.first_test_case.run_steps[0]
        self.assertIsInstance(step, DummyStep)

    def test_test_case_1_postconditions(self):
        self.assertEquals(len(self.first_test_case.postconditions), 1)
        condition_evaluator_pair = self.first_test_case.postconditions[0]
        self.assertIsInstance(condition_evaluator_pair.condition, BinaryCondition)
        self.assertEquals(condition_evaluator_pair.condition.name, "FalseCondition") 
        self.assertFalse(condition_evaluator_pair.condition.value) 
        self.assertIsInstance(condition_evaluator_pair.evaluator, StaticBooleanEvaluator)
        self.assertFalse(condition_evaluator_pair.evaluator.always_true)

    def test_test_case_1_tear_down(self):
        self.assertEquals(len(self.first_test_case.tear_down_steps), 1)
        step = self.first_test_case.tear_down_steps[0]
        self.assertIsInstance(step, DummyStep)

class TestParseSecondTestCase(unittest.TestCase):
    def setUp(self):
        parser = rospit.rospit_xml.get_test_suite_from_xml_string(test_xml)
        parser.parse()
        self.test_suite = parser.test_suite
        self.second_test_case = self.test_suite.test_cases[1]

    def test_test_case_2(self):
        self.assertEquals(self.second_test_case.name, "TC2")
