# 🧪 Guia de Teste Rápido - Sistema Autônomo de Freio com ABS

## Verificação Inicial de Compilação

✅ **Status**: Sistema compilado e executado com sucesso!

```bash
cargo build   # Compilação debug
cargo run     # Execução
cargo test    # Testes unitários
```

## 🎬 Cenas de Teste Recomendadas

### Teste 1: Freio Manual Progressivo (3-5 minutos)
**Objetivo**: Entender como o ABS reage a diferentes níveis de frenagem

1. Pressione `1` para **Modo Manual**
2. Pressione **↑** 5 vezes para acelerar até ~50 km/h
3. Pressione **→** lentamente aumentando freio de 10% em 10%
4. Observe:
   - ✓ Quando velocidade das rodas começa a cair (Skidding detectado)
   - ✓ Quando ABS é ativado (🟡 ABS nos status das rodas)
   - ✓ Ciclos ABS aumentando
   - ✓ Pressão pulsando entre 30-90%

**Resultado Esperado**: 
- Em ~30-40% de freio, ABS é ativado
- Rodas mantêm rotação enquanto desacelerando
- Distância de parada é otimizada

---

### Teste 2: Freio de Emergência (2-3 minutos)
**Objetivo**: Validar sistema em cenário crítico

1. Pressione `2` para **Emergency Brake**
2. Observe automaticamente:
   - Aceleração de 0 a ~100 km/h (3 segundos)
   - Aplicação de freio máximo (100%)
   - ABS ativado imediatamente
   - Múltiplos ciclos ABS

**Resultado Esperado**:
- Parada completa em ~5-6 segundos
- 8-12 ciclos de ABS
- Nenhuma roda deve travar (todas em 🟡 ou 🟢)

---

### Teste 3: Alta Velocidade (3-4 minutos)
**Objetivo**: Testar limites do sistema em alta velocidade

1. Pressione `3` para **High Speed**
2. Sistema acelera até velocidade máxima (~160-180 km/h)
3. Aplica frenagem moderada
4. Observe sensibilidade do ABS

**Resultado Esperado**:
- ABS ativo por mais tempo (maior diferença de velocidade)
- Mais ciclos ABS necessários
- Comportamento estável em alta velocidade

---

### Teste 4: Frenagens Repetidas (4-5 minutos)
**Objetivo**: Validar robustez do sistema

1. Pressione `4` para **Repeated Braking**
2. Sistema realiza 5 ciclos de:
   - Aceleração moderada
   - Frenagem
   - Recuperação
3. Analise consistência

**Resultado Esperado**:
- Ciclos ABS consistentes entre frenagens
- Sem travamento entre ciclos
- Distância de parada previsível

---

## 📊 Métricas para Análise

### Velocidade
- **Real**: Velocidade atual do veículo
- **Sensor**: Leitura com ruído (±0.5 km/h)
- **Diferença**: Desvio padrão do sensor

### Estado das Rodas
```
🟢 ROLLING    = Rodando normalmente (sem freio)
🟡 ABS        = Sistema ABS atuando (modulando pressão)
🔴 SKIDDING   = Roda travada (perigo!)
```

### Pressão de Freio
- **Escala**: 0% (sem freio) a 100% (freio máximo)
- **Com ABS**: Pulsa entre 30-90%
- **Sem ABS**: Constante

### Ciclos ABS
- **Incrementa**: A cada pulsação (8 Hz)
- **Indicador**: Efetividade do sistema
- **Esperado**: 1-2 ciclos por 0.1 segundo

---

## 🔍 Observações de Engenharia

### Comportamento Normal ✅
```
- Aceleração linear
- Freio sem travamento com ABS
- Rodas dianteiras mais sensíveis
- Pulsação regular a 8 Hz
- Velocidade sensor ~ velocidade real
```

### Comportamento Anômalo ⚠️
```
- Roda vermelha (🔴 SKIDDING) por >1 segundo
- Pressão não pulsando corretamente
- Diferença velocidade roda/veículo >15 km/h
- Aceleração negativa sem freio
```

---

## 🔧 Dados Técnicos para Análise

### Parâmetros de Simulação
```
Frequência ABS:         8 Hz (período: 0.125s)
Taxa de atualização:    60 FPS (dt: 0.016s)
Velocidade máxima:      200 km/h
Aceleração máxima:      5 m/s²
Desaceleração máxima:   10 m/s²
Ruído do sensor:        0.5 km/h σ
```

### Limites de Detecção
```
Travamento ativado:     Δv > 5 km/h E pressão > 0.3
Freio considerado:      Requisição > 0.1
Rodas afetadas:         Todas as 4 igualmente
```

### Dinâmica de Roda
```
Desaceleração: brake_force * 10.0 m/s²
Velocidade mínima: 0 km/h (sempre ≥ 0)
Atualização: Euler de 1ª ordem
```

---

## 📈 Gráfico de Teste: Velocidade vs Tempo

### Teste de Freio Normal (sem ABS)
```
200 km/h ┤
         │     ╱╲
150 km/h ┤    ╱  ╲
         │   ╱    ╲
100 km/h ┤  ╱      ╲
         │ ╱        ╲
 50 km/h ┤╱          ╲
         │             ╲
  0 km/h ┼──────────────┘
         └─────────────────── tempo (s)
         
Nota: Curva suave = sem travamento
```

### Teste com ABS Ativo
```
200 km/h ┤
         │     ╱╲
150 km/h ┤    ╱  ╲
         │   ╱    ╲
100 km/h ┤  ╱      ╲╱╲╱╲╱╲
         │ ╱        
 50 km/h ┤╱          
         │             
  0 km/h ┼──────────────
         └─────────────────── tempo (s)
         
Nota: Ondulações = pulsação ABS
```

---

## ✅ Checklist de Validação

- [ ] Compilação sem erros
- [ ] Interface visual renderiza corretamente
- [ ] Controles responsivos (aceleração/freio)
- [ ] Cenário Manual funciona
- [ ] Cenário Emergency Brake completa
- [ ] Cenário High Speed completa
- [ ] Cenário Repeated Braking completa
- [ ] Histórico de velocidade atualiza
- [ ] Pressão das rodas varia
- [ ] ABS ativa quando necessário
- [ ] Nenhuma roda travada por >1s com ABS
- [ ] Sensor mostra ruído realístico
- [ ] Pausa (SPACE) funciona
- [ ] Reset (R) funciona
- [ ] Saída (Q/ESC) funciona

---

## 🐛 Troubleshooting

### "Cannot find module" ao compilar
```bash
# Solução:
cargo clean
cargo build --release
```

### Interface não renderiza
```bash
# Verificar terminal:
- Mínimo 80x24 caracteres
- Suporte ANSI (Windows 10+)
- Não usar buffer compartilhado

# Solução:
Usar terminal nativo PowerShell ou WSL
```

### Comportamento imprevisível
```bash
# Reset do estado:
Pressione R para resetar
Ou
cargo run (nova instância)
```

---

## 📚 Recursos Adicionais

- [README.md](README.md) - Documentação completa
- [src/lib.rs](src/lib.rs) - Código da simulação
- [src/main.rs](src/main.rs) - Interface e controles

---

**Data**: 2026-08-11  
**Versão**: 0.1.0  
**Status**: ✅ Pronto para Testes
