import { Component, OnInit } from '@angular/core';

@Component({
  selector: 'app-voltagegraph',
  templateUrl: './voltagegraph.component.html',
  styleUrls: ['./voltagegraph.component.css']
})
export class VoltagegraphComponent implements OnInit {

  public barChartOptions: any = {
    scaleShowVerticalLines: false,
    scaleShowHorizontalLines: false,
    responsive: true,
    maintainAspectRatio: false,
    title: {
      display: true,
      fontColor: 'white',
      text: 'Engine Voltages',
      fontSize: 22,
    },
    scales: {
      yAxes: [{
        display: true,
        ticks: {
          max: 80,
          min: 0,
          suggestedMin: 0,    // minimum will be 0, unless there is a lower value.
          // OR //
          beginAtZero: true,   // minimum value will be 0.
          fontColor: 'white',
        }
      }],
      xAxes: [{
        display: true,
        ticks: {
          fontColor: 'white'
        },
        gridLines: {
          display: true,
          color: 'rgba(1, 1, 1, 1)'
        }
      }]
    }
  };
  public barChartLabels: string[] = ['Engine 1', 'Engine 2', 'Engine 3', 'Engine 4'];
  public barChartType = 'bar';
  public barChartLegend = false;
  public chartColors: any[] = [
    {
      backgroundColor: ['#003f5c', '#58508d', '#bc5090', '#ff6361']
    }];
  public barChartData: any[] = [
    { data: [65, 65, 65, 65, 65] },
  ];

  constructor() { }

  ngOnInit() {
  }

}
