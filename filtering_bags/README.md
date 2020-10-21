# Details about bag files
These bag files were all captures in a very short time window and are fairly similar
Modifications in the amount of noise and number of particles were done between these bags. 
The differences can be seen, though unfotunately we were unable to fine tune a rock sold particle filter.


### bagf2
- uses map ac1019
- 200 particles

Comments
- Very off at the end of the run, indicates some flaw in the process somewhere, or overly aggressive behavior

### bgag5 
- uses ac109 map
- 150 particles 

Comments
- tracks well
- Somehow it gets lost near the end :(

### bgf3
- uses ac109 map
- 150 particles 
- increased noise injection levels in theta values for resampled particles (you can see)

Comments
- tracks well
- doesn't condense particles fast or aggressively enough

### bgf4
- uses ac109 map
- 200 particles 

Comments
- Points don't condense and the filter gets off on the way back
- Overall very similar behavior to other bags


