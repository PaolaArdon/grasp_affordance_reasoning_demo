### Prerequisite
* Install the mln libraries for python with `pip install pracmln` 

### To run the mln with python
* To learn from a new database `python main.py`
* To query from the trained model with the reasoning affordance database `python query.py`

If you want to use your own database to train please check the .txt files and replicate the format for your DB.

### To run the program in a docker file
* Go into the mln_docker folder and run `docker build -t mln:gpu mln_docker .`
