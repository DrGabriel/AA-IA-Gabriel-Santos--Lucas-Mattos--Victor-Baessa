# pacmanAgents.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from pacman import Directions
from game import Agent
import random
import game
import util
from searchAgents import PositionSearchProblem

class LeftTurnAgent(game.Agent):
  "An agent that turns left at every opportunity"
  
  def getAction(self, state):
    legal = state.getLegalPacmanActions()
    current = state.getPacmanState().configuration.direction
    if current == Directions.STOP: current = Directions.NORTH
    left = Directions.LEFT[current]
    if left in legal: return left
    if current in legal: return current
    if Directions.RIGHT[current] in legal: return Directions.RIGHT[current]
    if Directions.LEFT[left] in legal: return Directions.LEFT[left]
    return Directions.STOP

class GreedyAgent(Agent):
  def __init__(self, evalFn="scoreEvaluation"):
    self.evaluationFunction = util.lookup(evalFn, globals())
    assert self.evaluationFunction != None
        
  def getAction(self, state):
    # Generate candidate actions
    legal = state.getLegalPacmanActions()
    if Directions.STOP in legal: legal.remove(Directions.STOP)
      
    successors = [(state.generateSuccessor(0, action), action) for action in legal] 
    scored = [(self.evaluationFunction(state), action) for state, action in successors]
    bestScore = max(scored)[0]
    bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
    return random.choice(bestActions)

class AstarPacman(game.Agent):

    def getPacmanSuccessors(self, legalActions, state):
      return [(state.generateSuccessor(self.index, action), action, state.generateSuccessor(self.index, action).getPacmanPosition()) for action in legalActions]

    def getAction(self, state):
        from util import PriorityQueue, Counter, manhattanDistance

        gridComidas = state.getFood()
        listaCapsulas = state.getCapsules()#Capsulas devem ter maior prioridade o pacman vai focar em buscar elas
        listaComidas = gridComidas.asList()
        listaObjetivo = []
        if len(listaCapsulas) > 0:
          listaObjetivo = listaCapsulas
        else:
          listaObjetivo = listaComidas
        
        mindist = 10000000#variavel auxiliar para pegar a menor distancia do pacman a comida
        comidaMaisProxima = (1,1)#variavel auxiliar para pegar a proxima comida
        for comida in listaObjetivo:
          dist = manhattanDistance(state.getPacmanPosition(), comida)
          if dist < mindist:
            mindist = dist
            comidaMaisProxima = comida#encontro a posicao da comida mais proxima

        caminhosPossiveis = PriorityQueue()
        caminhosPossiveis.push((state, []), 0)
        visitados = []#lista de estados ja visitados, para nao haver repeticoes
        
        while not caminhosPossiveis.isEmpty():
            estadoAtual, sequenciaDeAcoes = caminhosPossiveis.pop()#removo da heap a melhor sequencia de estados ate ento

            if estadoAtual.getPacmanPosition() == comidaMaisProxima:#encontrei meu objetivo
              if len(sequenciaDeAcoes) > 0:
                return sequenciaDeAcoes[0]
              else:
                return  estadoAtual.getLegalPacmanActions()[0]

            for proximoEstado, proximaAcao, proximaPosicao in self.getPacmanSuccessors(estadoAtual.getLegalActions(0), estadoAtual):
                if len(state.getCapsules())> 0:#se tem capsulas o pacman pode comer-las e nao se importar com os fantasmas no caminho
                  if proximaPosicao not in visitados:#verifico se o caminho nao eh repetido
                    visitados.append(proximaPosicao)  
                    acao = sequenciaDeAcoes + [proximaAcao]  
                    caminhosPossiveis.push((proximoEstado, acao), len(acao) + manhattanDistance(comidaMaisProxima, state.getPacmanPosition()))# f() = g() + h()                  
                elif not proximaPosicao in state.getGhostPositions():#verifico se o caminho nao possui fantasmas
                  if proximaPosicao not in visitados:#verifico se o caminho nao eh repetido
                    visitados.append(proximaPosicao)  
                    acao = sequenciaDeAcoes + [proximaAcao]  
                    caminhosPossiveis.push((proximoEstado, acao), len(acao) + manhattanDistance(comidaMaisProxima, state.getPacmanPosition()))# f() = g() + h()
                  

        return random.choice(state.getLegalPacmanActions())

def scoreEvaluation(state):
  return state.getScore()  
