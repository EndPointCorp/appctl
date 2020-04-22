pipeline {
  agent {
    dockerfile {
      args "-u 0"
    }
  }
  environment {
    APTLY_SERVER = credentials('aptly-server-url')
  }
  stages {
    stage('Test') {
      steps {
        sh "./scripts/run_tests.sh"
      }
    }
    stage('Build') {
      when {
        branch 'master'
      }
      steps {
        sh "./pack-debs"
      }
    }
    stage('Deploy') {
      when {
        branch 'mater'
      }
      steps {
        sh "./scripts/deploy.sh"
      }
    }
  }
}
