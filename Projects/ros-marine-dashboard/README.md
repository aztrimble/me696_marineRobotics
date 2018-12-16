# RosMarineDashboard

This project uses the following APIs:

+ [Angular](https://angular.io/)
+ [Material](https://material.angular.io/)
+ [Electron](https://electronjs.org/)
+ [OpenMapTiles Map Server](https://openmaptiles.com/server/#install)


Clone this repo and run the following command within the cloned directory:

```
npm install
```

# Development Commands:

To run the development environment of Angular. 
App can be seen within a browser at https://localhost:4200
```
npm run start
```

Will build the application in an Electron package that is for you current system. App will be placed within the respective folder inside the dist folder.
```
npm run dist
```

Will build the application in an Electron package for linux. App will be placed within the respective folder inside the dist folder.
```
npm run dist-linux
```

## MapTileServer
### Install Docker (Ubuntu)

    `$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -`
    `$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs)` stable"
    `$ sudo apt-get update`
    `$ sudo apt-get install -y docker-ce`
    
### Starting MapTile Server

    `$ sudo service docker start`

This command will start the maptile server. It will also install the application if you haven't already done so (requires internet connection the first time to install application)
 
    `$ docker  run  --rm  -it  -v  $(pwd):/data  -p  8080:80  klokantech/tileserver-gl ./maptiles/oahu.mbtiles`

### Oahu Map Tiles

The Oahu Map Tiles file is located within the maptile folder.

New or different map tiles can be download here: https://openmaptiles.com/downloads/planet/
