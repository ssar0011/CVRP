# CVRP
The capacitated vehicle routing problem (CVRP) is a VRP in which vehicles with limited carrying capacity need to pick up or deliver items at various locations. The items have a quantity, such as weight or volume, and the vehicles have a maximum capacity that they can carry. The problem is to pick up or deliver the items for the least cost, while never exceeding the capacity of the vehicles.

The following diagram illustrates a solution for a particular CVRP problem, where the different routes are labelled by their different colours and the nodes they must travel to (source: https://developers.google.com/optimization/routing/cvrp). 
<img width="781" alt="Screen Shot 2021-03-02 at 2 54 19 pm" src="https://user-images.githubusercontent.com/53897204/109594594-2c4ca300-7b67-11eb-858a-7e202c3bd728.png">

The data used to complete this project was downloaded from the Capacitated Vehicle Routing Problem Library (CVRPLIB) (http://vrp.atd-lab.inf.puc-rio.br/index.php/en/) and Google OR-Tools python package was used to solve the CVRP instances. 
