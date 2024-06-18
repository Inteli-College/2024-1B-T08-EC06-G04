---
sidebar_position: 3
custom_edit_url: null
title: "Mudanças finais no código"
---

# Introdução

Durante a sprint 5, foi realizada uma refatoração significativa no código do backend e na lógica de movimentação do robô, com foco na entrega da versão final do projeto. A equipe se concentrou em ajustar e otimizar os componentes existentes, com base nos testes que foram realizados. Esses testes identificaram vários pontos de melhoria, particularmente na implementação do botão de emergência (*Kill Switch*). A funcionalidade do botão de emergência é crucial para a segurança e controle do robô, permitindo a interrupção imediata de suas operações em situações de risco. No entanto, a implementação original estava incorreta, o que levou à necessidade de uma revisão completa desta funcionalidade.

Além disso, a refatoração do código do backend incluiu a melhoria na disposição do código e ajustes em funções que poderiam afetar o desempenho do sistema. A seguir, será apresentado o código anterior do botão de emergência e a versão atualizada que corrigiu os problemas identificados.

:::info
Os testes atualizados realizados pelo nosso grupo podem ser vistos em `Sprint 5` > `Documentação` > `Testes Realizados` > `Testes`
:::

---
## Kill Switch

O *Kill Switch*, ou botão de emergência, é um mecanismo de segurança essencial em sistemas de robótica e automação. Sua principal função é permitir a interrupção imediata e segura das operações do robô em caso de emergência ou situação de risco. Este recurso é fundamental para prevenir acidentes, danos ao equipamento e garantir a segurança de operadores e outros envolvidos no ambiente de operação.

A importância do *Kill Switch* reside em sua capacidade de interromper rapidamente todas as atividades do robô, evitando ações descontroladas que poderiam resultar em danos ou lesões. Em um ambiente onde a segurança é uma prioridade, a implementação correta deste mecanismo é vital. Ele deve ser capaz de parar o robô de forma instantânea e confiável, independentemente do estado em que o sistema se encontra.

Na implementação anterior, o método utilizado para o *Kill Switch* simplesmente fechava a conexão com o ROS, e esse método apresentava algumas falhas:
- Incerteza na Parada do Robô: Fechar a conexão com o ROS não garantia que o robô parasse imediatamente. O robô poderia continuar se movendo ou operando de maneira imprevisível até que a conexão fosse restabelecida ou até que os comandos em execução fossem naturalmente finalizados.
- Falta de Controle: A abordagem não fornecia um mecanismo de controle explícito sobre o estado de emergência, deixando o robô em um estado possivelmente inseguro.
- Não Conformidade com Melhores Práticas: A implementação não seguia as melhores práticas para sistemas de segurança em robótica, que exigem uma parada controlada e imediata.

Visto o exposto, a seguir, tem-se o processo de mudança do *Kill Switch*:

### Código Anterior

No código anterior, a tentativa de fechar a conexão com o ROS (Robot Operating System) era utilizada como uma forma de interromper as operações do robô. Abaixo está o trecho de código que implementava essa funcionalidade:

```python
const closeConnection = () => {
    if (ros) {
      ros.close();
      console.log('ROS connection closed.');
    }
}
```

Esta abordagem apresentava várias limitações. Simplesmente fechar a conexão com o ROS não garantia uma parada segura e controlada do robô, além de não seguir as melhores práticas para a implementação de um botão de emergência. A falta de uma chamada explícita a um serviço de parada de emergência significava que o robô poderia continuar em um estado indefinido ou inseguro até que a conexão fosse restabelecida.

### Código Atual

Para corrigir estas deficiências, foi desenvolvido um novo método que utiliza um serviço dedicado para a parada de emergência, conforme mostrado abaixo:

```python
const emergencyStop = () => {
    if (ros) {
      const emergencyStopService = new ROSLIB.Service({
        ros: ros,
        name: "/emergency_stop",
        serviceType: "std_srvs/srv/Empty",
      });

      const request = new ROSLIB.ServiceRequest({});

      emergencyStopService.callService(request, (result) => {
        console.log("Emergency stop service called.", result);
      });
    }
}
```

Nesta nova implementação, o método `emergencyStop` cria um serviço ROS chamado `/emergency_stop` do tipo `std_srvs/srv/Empty`. Ao chamar este serviço, garantimos que o comando de parada de emergência seja enviado corretamente ao robô, permitindo uma resposta imediata e controlada. Este serviço é projetado especificamente para lidar com situações de emergência, garantindo que todas as operações do robô sejam interrompidas de forma segura.