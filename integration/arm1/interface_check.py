"""Require the team's exact wire schema; do not replace another host_pkg."""
def validate_interface(Arm):
    expected={'Goal':{'command':'string'},'Result':{'success':'boolean','message':'string'},
              'Feedback':{'message':'string'}}
    for part,fields in expected.items():
        if getattr(Arm,part).get_fields_and_field_types()!=fields:
            raise RuntimeError(f'host_pkg/action/Arm {part} differs from agreed interface')
        getattr(Arm,part).__class__.__import_type_support__()


if __name__=='__main__':
    from host_pkg.action import Arm
    validate_interface(Arm)
    print('host_pkg/action/Arm: compatible')
