from compliant_servo_control.admittance_controller import AdmittanceController, WrenchSample


def test_controller_enters_contact_mode_from_axial_force():
    controller = AdmittanceController()

    correction = controller.compute(WrenchSample(fz=14.0))

    assert correction.contact is True
    assert correction.vz < 0.0
    assert correction.emergency_stop is False


def test_controller_stops_on_large_force_norm():
    controller = AdmittanceController()

    correction = controller.compute(WrenchSample(fx=50.0))

    assert correction.emergency_stop is True
    assert correction.vx == 0.0
    assert correction.vz == 0.0
