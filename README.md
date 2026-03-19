#### Python Version
This project is designed to use Python 3.14.x. E.g.:
```
$ python --version
3.14.3
```
It is easy to use `pyenv` or similar tools to install the correct version. Other versions are likely to work as long as the version number is 3.14 or higher.

#### Create a virtual environment
```
python -m venv venv
```
A folder called `venv` will appear in root folder of the project. It is ignored by default by git because of the entries in `.gitignore`.

The virtual environment must be activated to isolate dependencies rom the global interpreter:
```
. venv/bin/activate
```
If you are using VS Code, the environment should automatically activate when opening the workspace. See `CS5460-Project.code-workspace` for details. To verify which environment is active, check the location of the interpreter:
```
$ which python
/.../venv/bin/python`
```

The global interperter will have a different path to a non-local interpeter.


#### Install Dependencies
This should only be done once or whenever the list of dependencies changes:
```
pip install -r requirements.txt
```

#### Final Steps
The environment should be ready to use at this point.