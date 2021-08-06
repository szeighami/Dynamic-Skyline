# Dynamic Skyline Queries Using Result Materialization
This repository implements the algorithms in [1] for answering dynamic skyline queries using result materialization.

## To compile
Calling make compiles the program. It requires c++11 and has been tested with g++ 7.5.0. This creates the executable called skyline.

## To run
The program can be run with the following command:
```
./skyline <n> <d> <k> <m> <l> <alg> <data path>
```
Parameters n, d, k, m and l are defined the same way as [1]. Alg can be set to either G, in which case the greedy algorithm from [1] is used, or DP, where the dynamic programming algorithm is used. Finally, data path must contain the path to the dataset. The file must contain n lines, where each line contains the d space-separated attributes of the dataset for each record.

## Example
Calling
```
./skyline 100 2 1000 100 50 G db.txt
```
Runs the program for the sample dataset, db.txt, provided. The expected output is 

>find domination areas                                             
>found domination areas in 24ms              
>no components 2, n: 100, l: 50                                      
>Aggregating 1th component                                                                                                   
>aggreattion finished in  24ms                                             
>Building tree for component 0                                             
>Built tree in 3ms                                             
>Aggregating 2th component                                             
>aggreattion finished in  24ms                                             
>Building tree for component 1                                                                                                                         
>Built tree in 3ms  
>Total no. leaf nodes 2910                    
>Total time 211ms 






## References
[1] S. Zeighami, G. Ghinita, and C. Shahabi. "Secure Dynamic Skyline Queries Using Result Materialization." 2021 IEEE 37th International Conference on Data Engineering (ICDE), 2021.
