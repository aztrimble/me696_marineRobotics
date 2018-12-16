const html = __dirname + '/dist';

const port = 4000;
const apiUrl = '/api';

// Express
const bodyParser = require('body-parser');
const compression = require('compression');
const express = require('express');
var app = express();

app
  .use(compression())
  .use(bodyParser.json())
  // Static content
  .use(express.static(html))
  // Start server
  .listen(port, function () {
    console.log('Port: ' + port);
    console.log('Html: ' + html);
  });
