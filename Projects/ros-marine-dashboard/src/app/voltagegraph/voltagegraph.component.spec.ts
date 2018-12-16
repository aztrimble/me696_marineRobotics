import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { VoltagegraphComponent } from './voltagegraph.component';

describe('VoltagegraphComponent', () => {
  let component: VoltagegraphComponent;
  let fixture: ComponentFixture<VoltagegraphComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ VoltagegraphComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(VoltagegraphComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
