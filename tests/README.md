# Testing

## FAQs

### How to use a robot with another configuration than the default in my tests?

```python
@pytest.mark.parametrize('system', 'rb34', indirect=True)
def test_my_robot(system):
    # Your test code here
```
