import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ThreeModelComponent } from './three-model.component';

describe('ThreeModelComponent', () => {
  let component: ThreeModelComponent;
  let fixture: ComponentFixture<ThreeModelComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ThreeModelComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ThreeModelComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
