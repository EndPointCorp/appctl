pipeline {
  environment {
    APTLY_SERVER = credentials('aptly-server-url')
    BUILD_DEBS = "${env.BRANCH_NAME == "master" ? "true" : "false"}"
  }
  agent {
    dockerfile {
      args "-u 0 -v /var/lib/jenkins/.ssh:/root/ssh"
      additionalBuildArgs "--build-arg BUILD_DEBS=${env.BUILD_DEBS}"
    }
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
        input('debug this step')
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
