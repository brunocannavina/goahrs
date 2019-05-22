# GoAHRS

### Description
This packages wraps the implementation of Madgwick algorithm<br/>
to get orientation of an object based on accelerometer and gyroscope readings for Golang<br/>
More information about the algorithm can be found at http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

### Installation
- You must have Golang installed<br/>
- And not necessarily git<br/>
Go to C:\Go\src<br/>
`$ cd C:\Go\src`<br/>
Inside the folder execute the command:<br/>
`$ git clone https://github.com/brunocannavina/goahrs.git`<br/>
_(or you can simply download it from github)_<br/>
then, open the folder goahrs:<br/>
`$ cd goahrs`<br/>
and install the package:<br/>
`$ go install`<br/>

### Usage
In your code add the packages goahrs<br/>

```go
package main

import(
  "fmt"
  "goahrs"
)

func main(){

}
```
