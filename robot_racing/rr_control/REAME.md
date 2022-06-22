# rr_control

이 디렉토리는 rr_control 그룹의 디렉토리입니다. 
이 디렉토리에는 바로 패키지를 작성하지 않고 내부에 새로운 패키지 디렉토리를 생성하여 여러개의 패키지를 관리합니다.
이 그룹 목표는 차량의 control input을 만들어 내는 것입니다. 
단일 패키지가 아닌 이유는 역할을 분담한 lower level controller와 upper level controller가 서로 다른 주기로 작동할 수 있기 때문에 여러개의 컨트롤러의 구현을 염두해 두고 있기 때문입니다.