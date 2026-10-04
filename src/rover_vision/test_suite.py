import unittest
from test.test_tree_classifier import TestTreeVitalityClassification

def load_tests(loader, tests, pattern):
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestTreeVitalityClassification))
    return suite

if __name__ == '__main__':
    unittest.main()
